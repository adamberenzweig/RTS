// rfBee selftest
// optimized for least RAM usage
#include "TestIO.h"


#define numPins 6

#define IO_PD_4 4
#define IO_PB_1 9

#define IO_PB_0 8
#define IO_PD_7 7

#define IO_PC_4 18
#define IO_PC_5 19

#define IO_PD_6 6
#define IO_PD_5 5

#define IO_PC_0 14
#define IO_PC_1 15

#define IO_PC_2 16
#define IO_PC_3 17

#define IO_ADC_7 7


int TestIoPins(){
  byte pin[numPins*2]={ 
    IO_PD_4, 
    IO_PB_0,
    IO_PC_4,
    IO_PD_6,
    IO_PC_0,
    IO_PC_2,
    
    IO_PB_1,
    IO_PD_7,
    IO_PC_5,
    IO_PD_5,
    IO_PC_1,
    IO_PC_3,
    };  
  byte i=0;
  
  for (i=0;i<6;i++){
    pinMode(pin[i],INPUT);
    pinMode(pin[i+6],OUTPUT);
    digitalWrite(pin[i+6],HIGH);
  }
  for (i=0;i<6;i++){
    if (digitalRead(pin[i]) != HIGH)
      return ERR;
  }
  for (i=0;i<6;i++){
    pinMode(pin[i],OUTPUT);
    pinMode(pin[i+6],INPUT);
    digitalWrite(pin[i],HIGH);
  }
  for (i=0;i<6;i++){
    if (digitalRead(pin[i+6]) != HIGH)
        return ERR; 
  }
  if (analogRead(IO_ADC_7) < 500)
    return ERR; 
  
  return OK;
}

int TestIO(){
  int result=TestIoPins();
  if (result==OK){
    Config.set(CONFIG_HW_VERSION,HARDWAREVERSION);  // write the hardware version to eeprom
    Serial.println("IO ok");
  }
  else
    Serial.println("IO error");
  return result;
}
