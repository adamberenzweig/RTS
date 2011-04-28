RFBee v1.0

Copyright (c) 2010 Seedstudio.  All right reserved.

More information in www.seeedstudio.com
********************************************************


RFBee v1.01

1.Change method of receiving RF data from interupt to polling.
2.Add state check, avoiding RFBee into dead state.
3.Modify ATDR command return info.
2010/03/19 16:17 by Icing.

www.seeedstudio.com
*********************************************************

RFBee v1.1 alpha status !!

Main differences with the official 1.01 version:
- no need for ring buffer modifications, only a single global buffer used for transmission
- use of PROGMEM for CCx config stuff to save on RAM
- the ability to switch between the 5 CCx configs using the new ATCF command 
- more robust AT parameter checking (e.g. 999 is not allowed as Destination Address ;-))
- Serial AT handling separated from main RFbee stuff
- Clean isolation of the CCx code in a CCx class
- optional Interrupt Driven Receiver  (see #define INTERRUPT_RECEIVE) using a state machine
- no latch CS on every operation, but only during the init phase of the SPI
- use of a (simple) SPI class 
- code compiles both on 168 and 328 without modification

Hans Klunder

*********************************************************

RFBee v1.11 alpha

Can work now(not fully tested).

Bugs and Main changes:

1. The number of parameters to be saved to EEPROM is different. So it will be failed in Intializing. When use the v1.1a, comment the "if (Config.initialized() != OK)" first time downloading the program.

2. Use the sequence of v1.01 power on start up, select and deselect the CS in the read and write functions. When I use v1.1a, it failed, so I change it back.

3. In the readSerialData(), the processing of "+++" would not return immediately 
as it is less than Threshold(4) in v1.1. So when you type another "+++" or other command,the RFBee will return "ok"(entering command mode) and "error"(the command is cut). I did some changes making it work ok. But not fully tested.

4. In the using of txFisoSize(), the return value should be the number of bytes in TXFIFO but the empty space. However I'm not sure how to use it combining with the 
FIFOTHR.

5. In the loop(),the two ways of receiving RF data is almost the same. One is interuption mode, and the other is polling mode. However they both rely on the GDO0(pin2). So we need to initialize the pin2 as the INPUT in polling mode,and only when it is HIGH should we receiveData(). I think this is much better.

6. In the receiveData(),the processing of  potential RX overflows is not proper. When overflows,the STAE[2:0]=110, so I change it to "if((stat&0xF0)==0x60)", and it works ok.

7. In the transmitting part, in the loop(),we may need delay some time when Serial.avaiable()>0 to wait enugh data. And in the transmitData(), there are several bugs. (1) destAddress should be in front of srcAddress. (2) The lenght of writeBurst should be the length of serialData.(3) After strobe(TX), it should be given some time as the state would be changed to IDLE or RX in the loop() before the data is really transmitted.

8. I don't think it is a good way to use strobe(RX) in the loop all the time. Maybe make RFBee into proper mode by configuring MSCSM1 is better.When using v1.1a, when we set rfBeeMode = RECEIEVE_MODE, however we can still transmit data.
   
2010/5/22 Icing.
www.seeedstudio.com
**************************************************************************** 

Add some changes:
1. when in command mode, keep in it until receiving ATMD* command.
2. Only when in transmit or transceive mode, tranmit the serial buffer data otherwise flush it.
3. Remove the delay(5) in the loop() when serial data available.
4. Disable the RSSI and LQI data.
5. When two RFBees both in Transceive state and they are transmitting data, as if the interval is too small,there may error happen.For example, several packets are received but one interuption gernerated, so that the size read from the CCx_RXBYTES will be much more than the real buffer[0].

2010/6/1 Icing
www.seeedstudio.com
****************************************************************************

Add some changes:
1. Add ATSI* command to control if adding RSSI byte or not. ATSI0 disable RSSI, ATSI1 enable RSSI.
2. Modify the readSerialData(),so that the RFBee can properly transmit the last buffer data before entering command mode.
3. Default dest and source address are both set as 'A'(0x65).

2010/6/1 Icing
www.seeedstudio.com
***************************************************************************


