//  rfBeeCore.cpp core routines for the rfBee
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

#include "rfbeeCore.h"
#include <SleepControl.h>

// send data via RF
void transmitData(byte *txData,byte len, byte srcAddress, byte destAddress){
  DEBUGPRINT()
  byte stat;
  
  //Serial.println(len,DEC);
  CCx.Write(CCx_TXFIFO,len+2);
  CCx.Write(CCx_TXFIFO,destAddress);
  CCx.Write(CCx_TXFIFO,srcAddress);
  CCx.WriteBurst(CCx_TXFIFO,txData, len); // write len bytes of the serialData buffer into the CCx txfifo
  CCx.Strobe(CCx_STX);
#ifdef DEBUG
  txData[len]='\0';
  Serial.println((char *)txData);
#endif

}

// read available txFifo size and handle underflow (which should not have occured anyway)
byte txFifoFree(){
  DEBUGPRINT()
  byte stat;
  byte size;
  
  stat=CCx.Read(CCx_TXBYTES, &size);
  // handle a potential TX underflow by flushing the TX FIFO as described in section 10.1 of the CC 1100 datasheet
  if (stat & 0x80){
    CCx.Strobe(CCx_SFTX);
    stat=CCx.Read(CCx_TXBYTES,&size);
  }
#ifdef DEBUG
  Serial.println(CCx_FIFO_SIZE - size,DEC);
#endif
  return (CCx_FIFO_SIZE - size);
}

// receive data via RF, rxData must be at least CCx_PACKT_LEN bytes long
// On success, rxData points to the entire packet including the two address
// bytes at the beginning and the rssi and lqi bytes at the end.
int receiveData(byte *rxData, byte *len, byte *srcAddress, byte *destAddress, byte *rssi , byte *lqi){
  DEBUGPRINT()

  byte stat;
  int result;

  stat=CCx.Read(CCx_RXFIFO,len);
  result = CheckStatusForOverflow(stat);
  if (result != OK) {
    return result;
  }
#ifdef DEBUG
  Serial.print("length:");
  Serial.println(*len,DEC);
#endif
  // NOTE(madadam): Instead of CCx.Read for each non-payload byte of the packet,
  // we read the whole thing at once with ReadBurst, and parse it after.
  // It's less back-and-forth with the SPI, might be less error-prone. 
  // +2 for RSSI and LQI.
  if (*len + 2 > CCx_PACKT_LEN) {
    errNo = 1; // Invalid data size.
    return ERR;
  }
  stat = CCx.ReadBurst(CCx_RXFIFO, rxData, *len + 2);
  result = CheckStatusForOverflow(stat);
  if (result != OK) {
    return result;
  }
  *destAddress = rxData[0];
  *srcAddress = rxData[1];
  *rssi = rxData[*len];
  *rssi=CCx.RSSIdecode(*rssi);
  *lqi = rxData[*len + 1];

  // check checksum ok
  if ((*lqi & 0x80)==0){
    return NOTHING;
  }
  *lqi=*lqi & 0x7F; // strip off the CRC bit

  return OK;
}

int CheckStatusForOverflow(byte status) {
  // handle potential RX overflows by flushing the RF FIFO as described in section 10.1 of the CC 1100 datasheet
  if ((status & 0xF0) == 0x60){ //Modified by Icing. When overflows, STATE[2:0] = 110
     errNo=3; //Error RX overflow
     // Flush the RX buffer
     CCx.Strobe(CCx_SIDLE);
     delay(1);
     CCx.Strobe(CCx_SFRX);
     return ERR;
   }
   return OK;
}

/*
void lowPowerOn(){
   // Does setting additional registers make a difference? why not try? HOLY SHIT BALLS - 2 mA
  CCx.Write(CCx_MCSM2 ,0x01);
  CCx.Write(CCx_WOREVT1 ,0x28);
  CCx.Write(CCx_WOREVT0 ,0xA0);
  CCx.Write(CCx_WORCTRL ,0x38);
  CCx.Strobe(CCx_SRX);    //ccx into wake on radio state
  // pbbbblllttt...
  
  DEBUGPRINT()
  CCx.Write(CCx_WORCTRL,0x78);  // set WORCRTL.RC_PD to 0 to enable the wakeup oscillator
  CCx.Strobe(CCx_SWOR);
  sleepNow(SLEEP_MODE_IDLE);
  //CCx.Strobe(CCx_SIDLE);
}
*/
