//-------------------------------------------------------------------------
// This file implements platform specific functions while following an
// interface that the core code will be calling.
//-------------------------------------------------------------------------
#pragma once

#include "../common.h"
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

void platformInit();

int64_t platformGetMicrosecond();
int32_t platformGetMillisecond();
bool platformSleep(int32_t TimeMS);

lwSerialPort* platformCreateSerialPort();
