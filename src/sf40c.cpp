//----------------------------------------------------------------------------------------------------------------------------------
// LightWare SF40 ROS driver.
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define MAX_REV_POINTS 4096

enum lwScanRevState {
	LWSRS_WAIT,
	LWSRS_GATHERING
};

struct lwScanRev {
	float distances[MAX_REV_POINTS];
	uint8_t revId;
	uint32_t expectedPointCount;
	uint32_t pointCount;
	lwScanRevState state;
    ros::Time startTime;
};

sensor_msgs::LaserScan rosScanData;
lwScanRev scanRev;

void initScanRev(lwScanRev* Scan, sensor_msgs::LaserScan& ScanMsg, std::string FrameId) {
    Scan->revId = 0;
    Scan->expectedPointCount = 0;
    Scan->pointCount = 0;
	Scan->state = LWSRS_WAIT;

    ScanMsg.header.frame_id = FrameId;
    ScanMsg.angle_min = 0.0f;
    ScanMsg.angle_max = 360.0 / 180.0 * M_PI;
    ScanMsg.range_min = 0.0f;
    ScanMsg.range_max = 100.0f;
    ScanMsg.ranges.resize(0);
    ScanMsg.intensities.resize(0);
    
    ScanMsg.header.stamp = ros::Time::now();
    ScanMsg.angle_increment = 0.0f;
    ScanMsg.time_increment = 0.0f;
    ScanMsg.scan_time = 0.0f;
}

void generateScanPacket(lwScanRev* Scan, ros::Publisher& ScanPub, sensor_msgs::LaserScan& ScanMsg) {
	double scanDuration = 1.0f / 5.0f;
    
    ScanMsg.header.stamp = Scan->startTime;
    ScanMsg.angle_increment = (360.0 / 180.0 * M_PI) / Scan->pointCount;
    ScanMsg.time_increment = scanDuration / Scan->pointCount;
    ScanMsg.scan_time = scanDuration;
    ScanMsg.ranges.resize(Scan->pointCount);

    for (int i = 0; i < Scan->pointCount; ++i) {
        ScanMsg.ranges[i] = Scan->distances[Scan->pointCount - i - 1];
    }

    ScanPub.publish(ScanMsg);
}

int driverStart(lwSerialPort** Serial, const char* PortName, int32_t BaudRate) {
    platformInit();

    lwSerialPort* serial = platformCreateSerialPort();
    *Serial = serial;
	if (!serial->connect(PortName, BaudRate)) {
		// exitWithMessage("Could not establish Serial connection\n");
        ROS_ERROR("SF40C failed to connect to port %s", PortName);
        return 1;
	};

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName)) { return 1; }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) { return 1; }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;	
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) { return 1; }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber)) { return 1; }

	ROS_INFO("SF40C Model: %.16s", modelName);
	ROS_INFO("SF40C Hardware: %d", hardwareVersion);
	ROS_INFO("SF40C Firmware: %.16s (%d)", firmwareVersionStr, firmwareVersion);
	ROS_INFO("SF40C Serial: %.16s", serialNumber);

    return 0;
}

int driverScanStart(lwSerialPort* Serial) {
    // Set the output rate to full (20010 points per second). (Command 108: Output rate)
	if (!lwnxCmdWriteUInt8(Serial, 108, 0)) { return 1; }

	// Enable streaming of point data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(Serial, 30, 3)) { return 1; }

    return 0;
}

int driverScan(lwSerialPort* Serial) {
	// Wait for and process the streamed point data packets.
	// The incoming point data packet is Command 48: Distance output.
	lwResponsePacket response;

    if (lwnxRecvPacket(Serial, 48, &response, 1000)) {
        uint8_t 	alarmState = response.data[4];
        uint16_t 	pointsPerSecond = (response.data[6] << 8) | response.data[5];
        int16_t 	forwardOffset = (response.data[8] << 8) | response.data[7];
        int16_t 	motorVoltage = (response.data[10] << 8) | response.data[9];
        uint8_t 	revolutionIndex = response.data[11];
        uint16_t 	pointTotal = (response.data[13] << 8) | response.data[12];
        uint16_t 	pointCount = (response.data[15] << 8) | response.data[14];
        uint16_t 	pointStartIndex = (response.data[17] << 8) | response.data[16];
        uint16_t 	pointDistances[210];
        memcpy(pointDistances, response.data + 18, pointCount * 2);

        if (scanRev.state == LWSRS_WAIT) {
            if (pointStartIndex == 0) {
                scanRev.state = LWSRS_GATHERING;
                scanRev.pointCount = 0;
                scanRev.expectedPointCount = pointTotal;
                scanRev.revId = revolutionIndex;
                scanRev.startTime = ros::Time::now();
            }
        }

        if (scanRev.state == LWSRS_GATHERING) {
            if (scanRev.revId == revolutionIndex && scanRev.pointCount == pointStartIndex) {

                if (scanRev.pointCount + pointCount >= MAX_REV_POINTS) {
                    // NOTE: Too many points for a single revolution.
                    scanRev.state == LWSRS_WAIT;
                } else {
                    
                    for (int i = 0; i < pointCount; ++i) {
                        scanRev.distances[scanRev.pointCount + i] = (float)pointDistances[i] / 100.0f;
                    }

                    scanRev.pointCount += pointCount;

                    if (scanRev.pointCount == scanRev.expectedPointCount) {
                        // NOTE: All points for single revolution have been gathered.
                        scanRev.state = LWSRS_WAIT;
                        return 2;
                    }
                }
            } else {
                // NOTE: Points have been missed during this revolution.
                scanRev.state = LWSRS_WAIT;
            }
        }
    }

	return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sf40c");
    
    ros::NodeHandle n;
    ros::NodeHandle privateNode("~");
    ros::Publisher laserScanPub = n.advertise<sensor_msgs::LaserScan>("laserscan", 10);
    
    lwSerialPort* serial = 0;
    int32_t baudRate = 921600;

    std::string portName;
    privateNode.param(std::string("port"), portName, std::string("/dev/ttyUSB0"));
    std::string frameId;
    privateNode.param(std::string("frame_id"), frameId, std::string("laser"));

    ROS_INFO("Starting SF40C node");

    if (driverStart(&serial, portName.c_str(), baudRate) != 0) {
        ROS_ERROR("SF40C failed to start driver");
        return 1;
    }

    initScanRev(&scanRev, rosScanData, frameId);
    
    if (driverScanStart(serial) != 0) {
        ROS_ERROR("SF40C failed to start scan");
        return 1;
    }

    while (ros::ok()) {
        if (driverScan(serial) == 2) {
            generateScanPacket(&scanRev, laserScanPub, rosScanData);
        }
        ros::spinOnce();
    }

    ROS_INFO("SF40C shutdown");

    return 0;
}