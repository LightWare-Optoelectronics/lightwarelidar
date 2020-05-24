#pragma once

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "lwSerialPort.h"

#include "./linux/platformLinux.h"

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
void printHexDebug(uint8_t* Data, uint32_t Size);