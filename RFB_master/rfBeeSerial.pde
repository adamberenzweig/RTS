//  rfBeeSerial.pde serial interface to rfBee
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



#include "rfBeeSerial.h"

/*
void readSerialCmd(){
  DEBUGPRINT()  
  int result;
  char data;
  static byte pos=0;
  
  while(Serial.available()){
    result=NOTHING;
    data=Serial.read();
    serialData[pos++]=data; //serialData is our global serial buffer
    if (data == SERIALCMDTERMINATOR){
      if (pos > 3){ // we need 4 bytes
        result=processSerialCmd(pos);
      }
      else
        result=ERR;
      pos=0;
    }
    // check if we don't overrun the buffer, if so empty it
    if (pos > BUFFLEN){
      result=ERR;
      pos=0;
    }
    if (result == OK)
        Serial.println("ok");
    if (result == ERR)
        Serial.println("error");
  }
}
*/

/*
void readSerialData(){
  DEBUGPRINT()
  byte len;
  byte data;
  byte fifoSize=0;
  static byte plus=0;
  static byte pos=0;
  byte rfBeeMode;
 
  // insert any plusses from last round
  for(int i=pos; i< plus;i++) //be careful, i should start from pos, -changed by Icing
    serialData[i]='+';
  
  len=Serial.available()+plus+pos;
  if (len > BUFFLEN ) len=BUFFLEN; //only process at most BUFFLEN chars
  // check how much space we have in the TX fifo
  fifoSize=txFifoFree();// the fifoSize should be the number of bytes in TX FIFO
 
  if (len > fifoSize)  len=fifoSize;  // don't overflow the TX fifo
    
  for(byte i=plus+pos; i< len;i++){
    data=Serial.read();
    serialData[i]=data;  //serialData is our global serial buffer
    if (data == '+')
      plus++;
    else
      plus=0;
 
    if (plus == 3){
      len=i-2; // do not send the last 2 plusses
      plus=0;
      serialMode=SERIALCMDMODE;
      CCx.Strobe(CCx_SIDLE); 
      Serial.println("ok, starting cmd mode");
      break;  // jump out of the loop, but still send the remaining chars in the buffer 
    }
  }
  
  if (plus > 0)  // save any trailing plusses for the next round
    len-=plus;
   
  // check if we have more input than the transmitThreshold, if we have just switched to commandmode send  the current buffer anyway.
  if ((serialMode!=SERIALCMDMODE)  && (len < Config.get(CONFIG_TX_THRESHOLD))){
    pos=len;  // keep the current bytes in the buffer and wait till next round.
    return;
  }
  
  if (len > 0){
    rfBeeMode=Config.get(CONFIG_RFBEE_MODE);   
    //only when TRANSMIT_MODE or TRANSCEIVE,transmit the buffer data,otherwise ignore
    if( rfBeeMode == TRANSMIT_MODE || rfBeeMode == TRANSCEIVE_MODE )                             
        transmitData(serialData,len,Config.get(CONFIG_MY_ADDR),Config.get(CONFIG_DEST_ADDR)); 
    pos=0; // serial databuffer is free again.
  }
}
*/

// write a text label from progmem, uses less bytes than individual Serial.println()
//void writeSerialLabel(byte i){
//  char buffer[64];
//  strcpy_P(buffer, (char * )pgm_read_word(&(labels[i])));
//  Serial.println(buffer);
//}

//
void writeSerialError(){
  DEBUGPRINT()
  char buffer[64];
  strcpy_P(buffer, (char * )pgm_read_word(&(error_codes[errNo])));
  Serial.print("error: ");
  Serial.println(buffer);
}

// read data from CCx and write it to Serial based on the selected output format
/*
void writeSerialData(){
  DEBUGPRINT()
  byte rxData[CCx_PACKT_LEN];
  byte len;
  byte srcAddress;
  byte destAddress;
  byte rssi;
  byte lqi;
  int result;
  byte of;
 
  
  result=receiveData(rxData, &len, &srcAddress, &destAddress, &rssi , &lqi);
  
  if (result == ERR) {
      writeSerialError();
      return;
  }
  
  if (result == NOTHING)
    return;
// write to serial based on output format:
//  0: payload only
//  1: source, dest, payload
//  2: payload len, source, dest, payload, rssi, lqi
//  3: payload len, source, dest, payload,",", rssi (DEC),",",lqi (DEC)
  of=Config.get(CONFIG_OUTPUT_FORMAT);
  if(of == 3){
    Serial.print(len,DEC); // len is size of data EXCLUDING address bytes !
    Serial.print(',');
    // write source & destination
    Serial.print(srcAddress,DEC);
    Serial.print(',');
    Serial.print(destAddress,DEC);
    Serial.print(',');
    // write data 
    Serial.write(rxData,len); 
    Serial.print(',');
    // write rssi en lqi
    Serial.print(rssi,DEC);
    Serial.print(',');
    Serial.println(lqi,DEC);
    } 
  else{
    if( of > 1 ) 
       Serial.print(len); // len is size of data EXCLUDING address bytes !
    if( of > 0 ) {
      // write source & destination
      Serial.print(srcAddress);
      Serial.print(destAddress);
    }  
    // write data 
    Serial.write(rxData,len); 
    // write rssi en lqi
    if( of > 1 ) {
      Serial.print(rssi);
      Serial.print(lqi);
    } 
  }
}

*/

int setRFBeeModeWith(byte mode) {
   DEBUGPRINT()
  // CCx_MCSM1 is configured to have TX and RX return to proper state on completion or timeout
  switch (mode)
  {
  case IDLE_MODE:
    CCx.Strobe(CCx_SIDLE);
    break;
  case TRANSMIT_MODE:
    CCx.Strobe(CCx_SIDLE);
    delay(1);
    CCx.Write(CCx_MCSM1 ,   0x00 );//TXOFF_MODE->stay in IDLE
    CCx.Strobe(CCx_SFTX);
    // FIXME.  This remains mysterious.  On the slave, it seems I need to strobe
    // into TX mode here for things to work properly.  On the master, that
    // seemed to mess things up.
    //CCx.Strobe(CCx_STX);  FIXME
    break;
  case RECEIVE_MODE:
    CCx.Strobe(CCx_SIDLE);
    delay(1);
    CCx.Write(CCx_MCSM1 ,   0x0C );//RXOFF_MODE->stay in RX
    CCx.Strobe(CCx_SFRX);
    CCx.Strobe(CCx_SRX);
    break;
  case TRANSCEIVE_MODE:
    CCx.Strobe(CCx_SIDLE);
    delay(1);
    CCx.Write(CCx_MCSM1 ,   0x0F );//RXOFF_MODE and TXOFF_MODE stay in RX
    CCx.Strobe(CCx_SFTX);
    CCx.Strobe(CCx_SFRX);
    CCx.Strobe(CCx_SRX);
    break;
  case LOWPOWER_MODE:
    CCx.Strobe(CCx_SIDLE);
    break;  
  case SLEEP_MODE:
    CCx.Strobe(CCx_SIDLE);
    CCx.Strobe(CCx_SPWD);
    break;
  default:		
    break;
  }
  return OK;
}
