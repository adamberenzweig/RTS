//  Firmware for rfBee 
//  see www.seeedstudio.com for details and ordering rfBee hardware.

//  Copyright (c) 2010 Hans Klunder <hans.klunder (at) bigfoot.com>
//  Author: Hans Klunder, based on the original Rfbee v1.0 firmware by Seeedstudio
//  Version: June 18, 2010
//
//  This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with this program; 
//  if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA



#define FIRMWAREVERSION 11 // 1.1  , version number needs to fit in byte (0~255) to be able to store it into config
//#define FACTORY_SELFTEST
//#define INTERRUPT_RECEIVE
//#define DEBUG 


#include "debug.h"
#include "globals.h"
#include "Config.h"
#include "CCx.h"
#include "rfBeeSerial.h"

#ifdef FACTORY_SELFTEST
#include "TestIO.h"  // factory selftest
#endif

#define GDO0 2 // used for polling the RF received data

void setup(){
    if (Config.initialized() != OK) 
    {
      Serial.begin(9600);
      Serial.println("Initializing config"); 
#ifdef FACTORY_SELFTEST
      if ( TestIO() != OK ) 
        return;
#endif 
      Config.reset();
    }
    Serial.begin(pgm_read_dword(&baudRateTable[Config.get(CONFIG_BDINDEX)]));
    //Serial.print(Config.get(CONFIG_BDINDEX),DEC);
    rfBeeInit();
    Serial.println("ok");
}

void loop(){

  byte rfBeeMode;
  
  // CCx_MCSM1 is configured to have TX and RX return to IDLE on completion or timeout
  // so we need to explicitly enable RX mode.
  
  rfBeeMode=Config.get(CONFIG_RFBEE_MODE);   
  
  if ((rfBeeMode == RECEIVE_MODE) || (rfBeeMode == TRANSCEIVE_MODE))
    if (serialMode != SERIALCMDMODE){
      //INTERRUPT_GUARD( BUSY )
      CCx.Strobe(CCx_SRX);  
      //END_INTERRUPT_GUARD
    }
      
  if (Serial.available() > 0){
    if (serialMode == SERIALCMDMODE)
      readSerialCmd();
    else
      readSerialData();
  }
  
#ifdef USE_INTERRUPT_RECEIVE   
  if (state==RECV_WAITING)
     writeSerialData();
#else // polling mode
  if ( digitalRead(GDO0) == HIGH ) 
    writeSerialData();
#endif
}


void rfBeeInit(){
    DEBUGPRINT()
    
    CCx.PowerOnStartUp();
    loadSettings();
    serialMode=SERIALDATAMODE;
    
#ifdef USE_INTERRUPT_RECEIVE   
    state=IDLE;
    attachInterrupt(0, ISRVreceiveData, RISING);  //GD00 is located on pin 2, which results in INT 0
#else
    pinMode(GDO0,INPUT);// used for polling the RF received data
#endif 
}

// handle interrupt
#ifdef INTERRUPT_RECEIVE

void ISRVreceiveData(){
  DEBUGPRINT()
  
  if (state != IDLE)
    state=RECV_WAITING;
  else
    writeSerialData();
}

#endif





void loadSettings(){
  // load the appropriate config table
  byte cfg=Config.get(CONFIG_CONFIG_ID);
  CCx.Setup(cfg);  
  //CCx.ReadSetup();
  // and restore the config settings
  CCx.Write(CCx_ADDR, Config.get(CONFIG_MY_ADDR));
  CCx.Write(CCx_PKTCTRL1, (Config.get(CONFIG_ADDR_CHECK) | 0x04 ));
  CCx.setPA(cfg,Config.get(CONFIG_PAINDEX));
}
