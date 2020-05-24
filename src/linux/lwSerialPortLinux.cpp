#include "lwSerialPortLinux.h"

int32_t _convertBaudRate(int32_t BitRate) {
	switch (BitRate) {
		case 115200: { return B115200; }
		case 230400: { return B230400; }
		case 460800: { return B460800; }
		case 500000: { return B500000; }
		case 576000: { return B576000; }
		case 921600: { return B921600; }		
	}

	return B115200;
}

bool lwSerialPortLinux::connect(const char* Name, int BitRate) {
	_descriptor = -1;
	printf("Attempt com connection: %s\n", Name);
		
	_descriptor = open(Name, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (_descriptor < 0) {
		printf("Couldn't open serial port!\n");
		return false;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(_descriptor, &tty) != 0) {
		printf("Error from tcgetattr\n");
		return false;
	}

	BitRate = _convertBaudRate(BitRate);

	cfsetospeed(&tty, BitRate);
	cfsetispeed(&tty, BitRate);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= 0;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_iflag &= ~IGNBRK;
	tty.c_iflag &= ~ICRNL;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(_descriptor, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr\n");
		return false;
	}

	printf("Connected\n");

	return true;
}

bool lwSerialPortLinux::disconnect() {
	if (_descriptor >= 0) {
		close(_descriptor);
	}

	_descriptor = -1;

	return true;
}

int lwSerialPortLinux::writeData(uint8_t *Buffer, int32_t BufferSize) {
	if (_descriptor < 0) {
		printf("Can't write to null coms\n");
		return -1;
	}

	int writtenBytes = write(_descriptor, Buffer, BufferSize);

	if (writtenBytes != BufferSize)
	{
		printf("Could not send all bytes!\n");
		return -1;
	}

	return writtenBytes;
}

int32_t lwSerialPortLinux::readData(uint8_t *Buffer, int32_t BufferSize) {
	if (_descriptor < 0) {
		printf("Can't read from null coms\n");
		return -1;
	}

	errno = 0;
	int readBytes = read(_descriptor, Buffer, BufferSize);

	return readBytes;
}