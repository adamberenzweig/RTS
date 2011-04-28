/******* Reflecting the Stars *********
Radio Test - SLAVE CODE
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Utilities for debug printing.
**************************************/

#ifndef PRINT_UTIL_H 
#define PRINT_UTIL_H 1

#include "WProgram.h"

#ifdef DEBUG
    #define DPRINT( X ) \
    Serial.print( __FUNCTION__ ); \
    Serial.print( ": " ); \
    Serial.println( X );
#else
    #define DPRINT( X )
#endif

void DPrint(char* msg);
void DPrintln(char* msg);
void DPrintln();
void DPrintByte(char* label, byte b);
void DPrintByte(byte b);
void DPrintInt(char* label, int d);
void DPrintUL(char* label, unsigned long d);
void DPrintFloat(char* label, float f);
void DPrintBufHex(char* label, byte* data, int len);

void DebugPrintPacket(int result, byte* rxData, byte len,
                      byte src, byte dest, byte rssi, byte lqi);
void DFlashLeds(byte pin);
#endif
