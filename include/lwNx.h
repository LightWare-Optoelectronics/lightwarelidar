//----------------------------------------------------------------------------------------------------------------------------------
// The LWNX protocol is a binary based protocol for reading and writing data to LightWare devices.
//----------------------------------------------------------------------------------------------------------------------------------
#pragma once

#include "common.h"

#define PACKET_START_BYTE	0xAA
#define PACKET_TIMEOUT		200
#define PACKET_RETRIES		10

class lwResponsePacket {
	public:
		uint8_t data[1024];
		int32_t size;
		int32_t payloadSize;
		uint8_t parseState;

		lwResponsePacket();
};

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
// Create a CRC-16-CCITT 0x1021 hash of the specified data.
uint16_t lwnxCreateCrc(uint8_t* Data, uint16_t Size);

// Breaks an integer firmware version into Major, Minor, and Patch.
void lwnxConvertFirmwareVersionToStr(uint32_t Version, char* String);

//----------------------------------------------------------------------------------------------------------------------------------
// LWNX protocol implementation.
//----------------------------------------------------------------------------------------------------------------------------------
// Prepare a response packet for a new incoming response.
void lwnxInitResponsePacket(lwResponsePacket* Response);

// Waits to receive a packet of specific command id.
// Does not return until a response is received or a timeout occurs.
bool lwnxRecvPacket(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response, uint32_t TimeoutMs);

// Returns true if full packet was received, otherwise finishes immediately and returns false while waiting for more data.
bool lwnxRecvPacketNoBlock(lwSerialPort* Serial, uint8_t CommandId, lwResponsePacket* Response);

// Composes and sends a packet.
void lwnxSendPacketBytes(lwSerialPort* Serial, uint8_t CommandId, uint8_t Write, uint8_t* Data, uint32_t DataSize);

// Handle both the sending and receving of a command. 
// Does not return until a response is received or all retries have expired.
bool lwnxHandleManagedCmd(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize, bool Write = false, uint8_t* WriteData = NULL, uint32_t WriteSize = 0);

//----------------------------------------------------------------------------------------------------------------------------------
// Command functions.
//----------------------------------------------------------------------------------------------------------------------------------
// Issue read commands.
bool lwnxCmdReadInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t* Response);
bool lwnxCmdReadInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t* Response);
bool lwnxCmdReadInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t* Response);

bool lwnxCmdReadUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response);
bool lwnxCmdReadUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t* Response);
bool lwnxCmdReadUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t* Response);

bool lwnxCmdReadFloat(lwSerialPort* Serial, uint8_t CommandId, float* Response);

bool lwnxCmdReadString(lwSerialPort* Serial, uint8_t CommandId, char* Response);
bool lwnxCmdReadData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Response, uint32_t ResponseSize);

// Issue write commands.
bool lwnxCmdWriteInt8(lwSerialPort* Serial, uint8_t CommandId, int8_t Value);
bool lwnxCmdWriteInt16(lwSerialPort* Serial, uint8_t CommandId, int16_t Value);
bool lwnxCmdWriteInt32(lwSerialPort* Serial, uint8_t CommandId, int32_t Value);

bool lwnxCmdWriteUInt8(lwSerialPort* Serial, uint8_t CommandId, uint8_t Value);
bool lwnxCmdWriteUInt16(lwSerialPort* Serial, uint8_t CommandId, uint16_t Value);
bool lwnxCmdWriteUInt32(lwSerialPort* Serial, uint8_t CommandId, uint32_t Value);

bool lwnxCmdWriteFloat(lwSerialPort* Serial, uint8_t CommandId, float Value);

bool lwnxCmdWriteString(lwSerialPort* Serial, uint8_t CommandId, char* String);
bool lwnxCmdWriteData(lwSerialPort* Serial, uint8_t CommandId, uint8_t* Data, uint32_t DataSize);