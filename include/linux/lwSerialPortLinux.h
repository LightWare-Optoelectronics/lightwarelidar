#pragma once

#include "platformLinux.h"

class lwSerialPortLinux : public lwSerialPort {
	private:
		int32_t _descriptor;

	public:
		bool connect(const char* Name, int BitRate);
		bool disconnect();
		int writeData(uint8_t *Buffer, int32_t BufferSize);
		int32_t readData(uint8_t *Buffer, int32_t BufferSize);
};