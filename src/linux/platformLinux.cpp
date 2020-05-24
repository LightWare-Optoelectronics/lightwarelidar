#include "platformLinux.h"
#include "lwSerialPortLinux.h"

void platformInit() { }

int64_t platformGetMicrosecond() {
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);

	return time.tv_sec * 1000000 + time.tv_nsec / 1000;
}

int32_t platformGetMillisecond() {
	return (platformGetMicrosecond() / 1000);
}

bool platformSleep(int32_t TimeMS) {
	usleep(TimeMS * 1000);
	return true;
};

lwSerialPort* platformCreateSerialPort() {
	return new lwSerialPortLinux();
}
