# LightWare Optoelectronics ROS driver package

## SF40C Node

This node interfaces with an SF40/C and passes scan data to the laserscan topic.

Note: The SF40C should be configured to run at a baud rate of 921600. This is the default factory setting.

### Published topics
/laserscan (sensor_msgs/LaserScan)

A full 360 degrees of scan data is published to this topic at around 5Hz.

### Parameters
~port (string, default: /dev/ttyUSB0)

The communications port used to interface with the SF40C.

~frame_id (string, default: laser)

The transformation frame.



