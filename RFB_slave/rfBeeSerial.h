//  rfBeeSerial.h serial interface to rfBee
//  see www.seeedstudio.com for details and ordering rfBee hardware.

//  Copyright (c) 2010 Hans Klunder <hans.klunder (at) bigfoot.com>
//  Author: Hans Klunder, based on the original Rfbee v1.0 firmware by Seeedstudio
//  Version: July 14, 2010
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef RFBEESERIAL_H
#define RFBEESERIAL_H 1

#include "debug.h"
#include "globals.h"
#include "CCx.h"
#include "rfBeeCore.h"
#include <avr/pgmspace.h>

#define BUFFLEN CCx_PACKT_LEN
#define SERIALCMDMODE 1
#define SERIALDATAMODE 0
#define SERIALCMDTERMINATOR 13  // use <CR> to terminate commands

//void readSerialCmd();
//void readSerialData();
void writeSerialError();

int setRFBeeModeWith(byte mode);


byte serialData[BUFFLEN+1]; // 1 extra so we can easily add a /0 when doing a debug print ;-)
byte serialMode;
//volatile int sleepCounter;
static int sleepCounter = 1000;

// error codes and labels
byte errNo;

static char error_0[] PROGMEM="no error";
static char error_1[] PROGMEM="RX invalid data size";
static char error_2[] PROGMEM="RX invalid data";
static char error_3[] PROGMEM="RX buffer overflow";
static char error_4[] PROGMEM="CRC check failed";

static char *error_codes[] PROGMEM={
  error_0,
  error_1,
  error_2,
  error_3,
};


long baudRateTable[] PROGMEM= {9600,19200,38400,115200};

// operating modes, used by ATMD

#define IDLE_MODE 0
#define TRANSMIT_MODE 1     
#define RECEIVE_MODE 2 
#define TRANSCEIVE_MODE 3
#define LOWPOWER_MODE 4
#define SLEEP_MODE 5  

#ifdef INTERRUPT_RECEIVE
volatile enum state

#endif

#endif
