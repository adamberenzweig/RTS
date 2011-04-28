/******* Reflecting the Stars *********
Radio Test - SLAVE CODE
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-15
----------
Utilities for debug printing.
**************************************/

#include "WProgram.h"
#include "PrintUtil.h"

// TODO: Add debug_level to all of this.

// FIXME: this isn't carried over from the main pde file, why not?
#define DEBUG

void DPrint(char* msg) {
#ifdef DEBUG
  Serial.print(msg);
#endif
}

void DPrintln(char* msg) {
#ifdef DEBUG
  Serial.println(msg);
#endif
}

void DPrintln() {
#ifdef DEBUG
  Serial.println();
#endif
}

void DPrintByte(char* label, byte b) {
#ifdef DEBUG
  Serial.print(label);
  Serial.print(": ");
  Serial.print(b, DEC);
#endif
}

void DPrintByte(byte b) {
#ifdef DEBUG
  Serial.print(b, DEC);
#endif
}

void DPrintInt(char* label, int d) {
#ifdef DEBUG
  Serial.print(label);
  Serial.print(": ");
  Serial.print(d, DEC);
#endif
}

void DPrintUL(char* label, unsigned long d) {
#ifdef DEBUG
  Serial.print(label);
  Serial.print(": ");
  Serial.print(d, DEC);
#endif
}

void DPrintFloat(char* label, float f) {
#ifdef DEBUG
  Serial.print(label);
  Serial.print(": ");
  Serial.print(f);
#endif
}

void DPrintBufHex(char* label, byte* data, int len) {
#ifdef DEBUG
  Serial.print(label);
  Serial.print(": ");
  for (int i = 0; i < len; ++i) {
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
#endif
}

void DebugPrintPacket(int result, byte* rxData, byte len, byte srcAddress, byte destAddress, byte rssi, byte lqi) {
#ifdef DEBUG
  DPrintInt("result", result);
  DPrintByte(" len", len);
  DPrintBufHex(" data", rxData, len);
  DPrintByte(" src", srcAddress);
  DPrintByte(" dest", destAddress);
  DPrintByte(" rssi", rssi);
  DPrintByte(" lqi", lqi);
  DPrintln();
#endif
}

void DFlashLeds(byte pin) {
#ifdef DEBUG
 analogWrite(pin, 255);
 analogWrite(pin, 0);
#endif
}
