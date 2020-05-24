#include "lwNx.h"

lwResponsePacket::lwResponsePacket() : size(0), payloadSize(0), parseState(0) { }

uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size)
{
	uint16_t crc = 0;

	for (uint32_t i = 0; i < Size; ++i)
	{
		uint16_t code = crc >> 8;
		code ^= Data[i];
		code ^= code >> 4;
		crc = crc << 8;
		crc ^= code;
		code = code << 5;
		crc ^= code;
		code = code << 7;
		crc ^= code;
	}

	return crc;
}

void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String) {
	uint32_t major = (Version >> 16) & 0xFF;
	uint32_t minor = (Version >> 8) & 0xFF;
	uint32_t patch = (Version >> 0) & 0xFF;

	sprintf(String, "%d.%d.%d", major, minor, patch);
}

void lwnxInitResponsePacket(lwResponsePacket* Response) {
	Response->size = 0;
	Response->payloadSize = 0;
	Response->parseState = 0;
}

bool lwnxParseData(lwResponsePacket* Response, uint8_t Data) {
	if (Response->parseState == 0) {
		if (Data == PACKET_START_BYTE) {
			Response->parseState = 1;
			Response->data[0] = PACKET_START_BYTE;
		}
	} else if (Response->parseState == 1) {
		Response->parseState = 2;
		Response->data[1] = Data;
	} else if (Response->parseState == 2) {
		Response->parseState = 3;
		Response->data[2] = Data;
		Response->payloadSize = (Response->data[1] | (Response->data[2] << 8)) >> 6;
		Response->payloadSize += 2;
		Response->size = 3;

		if (Response->payloadSize > 1019) {
			Response->parseState = 0;
			printf("Packet too long\n");
		}
	} else if (Response->parseState == 3) {
		Response->data[Response->size++] = Data;

		if (--Response->payloadSize == 0) {
			Response->parseState = 0;
			uint16_t crc = Response->data[Response->size - 2] | (Response->data[Response->size - 1] << 8);
			uint16_t verifyCrc = lwnxCreateCrc(Response->data, Response->size - 2);

			if (crc == verifyCrc) {
				Response->parseState = 0;
				return true;
			} else {
				Response->parseState = 0;
				printf("Packet has invalid CRC\n");
			}
		}
	}

	return false;
}

bool lwnxRecvPacketNoBlock(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response) {
	uint8_t byte = 0;
	int32_t bytesRead = Serial->readData(&byte, 1);

	if (bytesRead != 0) {
		if (lwnxParseData(Response, byte)) {
			int8_t cmdId = Response->data[3];
			
			if (cmdId == CommandId) {
				return true;
			}
		}
	}

	return false;
}

bool lwnxRecvPacket(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs) {
	lwnxInitResponsePacket(Response);

	uint32_t timeoutTime = platformGetMillisecond() + TimeoutMs;
	uint8_t byte = 0;
	int32_t bytesRead = 0;

	while ((platformGetMillisecond() < timeoutTime) && (bytesRead = Serial->readData(&byte, 1)) != -1) {
		if (bytesRead > 0) {
			if (lwnxParseData(Response, byte)) {
				int8_t cmdId = Response->data[3];
				// printf("Got packet: %d\n", cmdId);
				// printf("Recv ");
				// printHexDebug(Response->data, Response->size);
				if (cmdId == CommandId) {
					return true;
				}
			}
		}
	}

	return false;
}

void lwnxSendPacketBytes(lwSerialPort* Serial, uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize) {
	uint8_t buffer[1024];
	uint32_t payloadLength = 1 + DataSize;
	uint16_t flags = (payloadLength << 6) | (Write & 0x1);
	
	buffer[0] = PACKET_START_BYTE;					// Start byte.
	buffer[1] = ((uint8_t*)&flags)[0];				// Flags low.
	buffer[2] = ((uint8_t*)&flags)[1];				// Flags high.
	buffer[3] = CommandId;							// Payload: Command ID.
	memcpy(buffer + 4, Data, DataSize);				// Payload: Data.
	uint16_t crc = lwnxCreateCrc(buffer, 4 + DataSize);
	buffer[4 + DataSize] = ((uint8_t*)&crc)[0];		// Checksum low.
	buffer[5 + DataSize] = ((uint8_t*)&crc)[1];		// Checksum high.

	// print("Send ");
	// printHexDebug(buffer, 6 + DataSize);

	Serial->writeData(buffer, 6 + DataSize);
}

bool lwnxHandleManagedCmd(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, bool Write, uint8_t* WriteData, uint32_t WriteSize) {
	int32_t attempts = PACKET_RETRIES;

	while (attempts--) {
		lwnxSendPacketBytes(Serial, CommandId, Write, WriteData, WriteSize);

		lwResponsePacket response;
		
		if (lwnxRecvPacket(Serial, CommandId, &response, PACKET_TIMEOUT)) {
			memcpy(Response, response.data + 4, ResponseSize);
			return true;
		}
	}

	return false;
}

bool lwnxCmdReadInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 1);
}

bool lwnxCmdReadInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 2);
}

bool lwnxCmdReadInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 4);
}

bool lwnxCmdReadUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 1);
}

bool lwnxCmdReadUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 2);
}

bool lwnxCmdReadUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 4);
}

bool lwnxCmdReadString(lwSerialPort* Serial, uint8_t CommandId, char* Response) {
	return lwnxHandleManagedCmd(Serial, CommandId, (uint8_t*)Response, 16);
}

bool lwnxCmdReadData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize) {
	return lwnxHandleManagedCmd(Serial, CommandId, Response, ResponseSize);
}

bool lwnxCmdWriteInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 1);
}

bool lwnxCmdWriteInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 2);
}

bool lwnxCmdWriteInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 4);
}

bool lwnxCmdWriteUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 1);
}

bool lwnxCmdWriteUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 2);
}

bool lwnxCmdWriteUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t Value) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)&Value, 4);
}

bool lwnxCmdWriteString(lwSerialPort* Serial, uint8_t CommandId, char* String) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, (uint8_t*)String, 16);
}

bool lwnxCmdWriteData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Data, uint32_t DataSize) {
	return lwnxHandleManagedCmd(Serial, CommandId, NULL, 0, true, Data, DataSize);
}