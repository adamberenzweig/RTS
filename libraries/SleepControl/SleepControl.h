/******* Reflecting the Stars *********
SleepControl
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Sleep and power control functions.
**************************************/

#ifndef SLEEP_CONTROL_H 
#define SLEEP_CONTROL_H 1

#include "WProgram.h"

// You must call this before using any of the sleep functions in here.
void InitSleepControl();

// Returns how long we slept for, in ms.
unsigned long sleepWithTimeout(unsigned long ms_to_sleep);

void setWatchdogInterrupt(byte enable, byte wdp_setting);
void sleepNow(byte mode);

#endif  // SLEEP_CONTROL_H
