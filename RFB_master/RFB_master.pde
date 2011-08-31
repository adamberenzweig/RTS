/******* Reflecting the Stars *********
RFBee MASTER CODE
Authors: Adam Berenzweig
Date: September 16, 2010
----------
Get RtsMessages from the serial port and transmit them over the radio.

Several testing modes: 
- continuously transmitting hard-coded messages.
- reading messages from the serial port and transmitting.
**************************************/

#include <DayCycle.h>
#include <Flash.h>
#include <MemoryFree.h>
#include <MessageTimer.h>
#include <PrintUtil.h>
#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <RtsUtil.h>
#include <SleepControl.h>

/***************** Early Definitions / Variables ******************/
#define FIRMWAREVERSION 11 // 1.1  , version number needs to fit in byte (0~255) to be able to store it into config

#define RTS_ID 1    // The Unique ID of this RFBee.
byte srcAddress = RTS_ID;

// Address to send to.  Use 0 for broadcast.
// Actually the pucks ignore this since we set them to global-receive mode (address check off).
byte destAddress = 2;

char versionblurb[] = "v.1.0 - MASTER";

//#define INTERRUPT_RECEIVE
//#define DEBUG 
/*****************************************************/

/************************* Includes ******************/
#include "debug.h"
#include "globals.h"
#include "CCx.h"
#include "rfBeeSerial.h"
/*****************************************************/


#define BLUE_PIN 5
#define WHITE_PIN 6
#define SOLAR_PIN 7
#define VOLTAGE_READ_PIN 3

#define GDO0 2 // used for polling the RF received data

#define STATUS_INTERVAL_MS 10000UL
unsigned long last_status_report = 0;
// How many messages we've sent.
unsigned long tx_counter = 0;
// Last time we sent.
unsigned long last_tx_time = 0;

// Voltage thresholder settings.
#define VOLTAGE_THRESHOLD_FRACTION .85
#define VOLTAGE_HISTERESIS_FRACTION .05
#define VOLTAGE_WINDOW_LEN 10

// MESSAGE_MODE controls how we get messages before sending them out.
enum MESSAGE_MODES {
  // Cycle through the test_messages.
  TEST_CYCLE,

  // Read messages from the serial port.
  MESSAGE_SERIAL,
};
//#define MESSAGE_MODE MESSAGE_SERIAL
#define MESSAGE_MODE TEST_CYCLE

// FIXME:  Getting short on SRAM again because of all the message
// strings.  Put them in progmem.

// Test mode parameters:
#define NUM_MSG_TO_SEND -1   // -1 to go forever

MessageTimer message_timer_;

//FLASH_STRING(fast_blue_odd, "TWK 245 10 0 255");
//FLASH_STRING(sleep_15, "SLEEP 15 60");
FLASH_STRING(all_off, "OFF");
FLASH_STRING(sleep_one_hour, "SLEEP 60 60");
FLASH_STRING(fast_blue_all, "TWK 245 10 0");

// Messages to cycle through in test mode 1.
TimedMessage test_messages[] = {
//  { 240000, "TWK 215 60 0"},  // Sparse blue twinkle.
//  { 240000, "TWK 215 60 1"},  // Sparse white twinkle.
//  { 0,      "SW 20 1 0 2 16"},     // Star wars both ways.
//  { 240000, "TWK 245 10 1"},  // fast white twinkle

//  { 60000, &sleep_one_hour },
//  { 60000, &all_off },
  { 60000, &fast_blue_all },
};

TimedMessage bedtime_sequence[] =  {
  { 4000, &all_off  },
  { 0,    &sleep_one_hour },  // Sleep one hour, indefnitely.
};

// The index of the current test message.
byte current_message_ = 0;  // FIXME moved inside MessageTimer?
// Current state of the radio.
byte radio_mode_ = TRANSMIT_MODE;

bool should_listen_for_status_response_ = false;

// TODO(madadam): PROGMEM.
char* all_attention_message = "ATTN";
// TODO(madadam):
// When I introduced the ATTENTION command, I was worried that pucks
// could get stuck at attention if they miss the AT_EASE at the end of the
// star wars routine.  To be safe, every time we change messages, we sent 
// AT_EASE.  Pucks already at ease will either miss it, or not care, and
// pucks stuck at attention have a chance to recover.  If this still matters,
// we can intersperse short AT_EASE into the test messages.
//char* at_ease_message = "ATEZ";
char* all_off_message = "OFF";

// Work around a bug that causes the RFBee to stop transmitting after a while.
// Resetting the rfbee radio mode periodically seems to help.
#define RADIO_RESET_INTERVAL_PACKETS 1000UL

// Time after transmitting to wait before doing anything else with the radio.
// We don't want to switch to RX mode or try to send another packet before the
// radio has finished transmitting.  There's probably a more principled way to
// have the radio let us know when it's done, but this seems to be effective.
// Things go bad if this is less than 60ms.
// Must be less than PACKET_PERIOD_MS
#define POST_TRANSMIT_REST_MS 75



//================this is for Master RFBee======================

// TODO(madadam): I think the EEPROM isnt being used right throughout this code.
// Why set it from a #define and then just read it back again?  Only need to
// use it if we can't hardcode the config data and instead want to configure
// with a serial port or similar.

void setup(){
  // For the voltage reading pin. NOTE: requires cutting VREF from V+.
  analogReference(INTERNAL);

  pinMode(SOLAR_PIN, INPUT);
  pinMode(VOLTAGE_READ_PIN, INPUT);
  
  if (RTS_MESSAGE_SIZE > CCx_PACKT_LEN) {
    DPrintln("Error: RTS_MESSAGE_SIZE too long.");
  }
  
  Serial.begin(9600);

  rfBeeInit();
  InitSleepControl();
  Serial.println(versionblurb);
  Serial.print("ID: ");
  Serial.println(RTS_ID, DEC);
  RunStartupSequence();

  InitActiveCycle();
}

void InitActiveCycle() {
  byte num_msgs = (byte)(sizeof(test_messages) / sizeof(TimedMessage));
  message_timer_.StartWithMessages(test_messages, num_msgs);
}

void InitSleepCycle() {
  byte num_msgs = (byte)(sizeof(bedtime_sequence) / sizeof(TimedMessage));
  message_timer_.StartWithMessages(bedtime_sequence, num_msgs);
}

DayCycleWithSolar day_cycle_(SOLAR_PIN);

void loop() {
  unsigned long now = millis();

  // Only use the day/night cycles in test_messages mode.
  if (MESSAGE_MODE == TEST_CYCLE) {
    if (day_cycle_.CheckForTransition(now)) {
      if (day_cycle_.state() == ACTIVE) {
        InitActiveCycle();
      } else if (day_cycle_.state() == SLEEPING) {
        InitSleepCycle();
      } else if (day_cycle_.state() == STANDBY) {
        // In standby, we don't transmit, but set this for safety.
        SetMessage(all_off_message);
      }
    }
  }

  if (MESSAGE_MODE == MESSAGE_SERIAL) {
    TryReadMessageFromSerial();
  }

  if (day_cycle_.state() == ACTIVE && MESSAGE_MODE == TEST_CYCLE) {
    message_timer_.MaybeChangeMessage(now);
  }
  if (day_cycle_.state() != STANDBY) {
    MaybeSendMessage(now, PACKET_PERIOD_MS);
  }
  if (should_listen_for_status_response_) {
    // 100 == kStatusListenTimeMs.
    WaitToReceiveStatusUntilTimeout(100);
  }

  MaybeReportStatus(now);
}

inline void SetMessage(const char* message) {
  message_timer_.SetMessageFromString(message);
}

void TryReadMessageFromSerial() {
  if (ReadMessageStringFromSerial(&Serial, serialData, BUFFLEN)) {
    Serial.print("ack ");
    Serial.println((char*)serialData);
    message_timer_.SetMessageFromString((const char*)serialData);
  }
}

void MaybeSendMessage(unsigned long now, int period_ms) {
  if ((NUM_MSG_TO_SEND < 0 || tx_counter < NUM_MSG_TO_SEND) &&
       IsTimerExpired(now, &last_tx_time, period_ms)) {
    SendOneMessage(message_timer_.message_data());

    // If we just sent a SEND_STATUS command, we should listen for a
    // response soon.
    should_listen_for_status_response_ =
      (message_timer_.rts_message().command() == RTS_SEND_STATUS);
    
    // In prod, attached to the Mega, there won't be any LEDs, but it's probably
    // safe to do this anyway since the pins won't be connected to anything.
    DFlashLeds(WHITE_PIN);
  }
}

void SendOneMessage(byte* message_data) {
  if (radio_mode_ != TRANSMIT_MODE ||
      (RADIO_RESET_INTERVAL_PACKETS > 0 &&
       tx_counter % RADIO_RESET_INTERVAL_PACKETS == 0)) {
    SetRadioMode(TRANSMIT_MODE);
  }
  transmitData(message_data, RTS_MESSAGE_SIZE, srcAddress, destAddress);
  // NOTE: This slows things down a lot!  If you want fast performance for
  // Star Wars mode, don't do this here:
  //DebugPrintPacketTx(message_data, RTS_MESSAGE_SIZE, srcAddress, destAddress);
  tx_counter++;
  delay(POST_TRANSMIT_REST_MS);
}

/*
void SendNMessages(int n) {
  for (int i = 0; i < n; ++i) {
    SendOneMessage(message_timer_.message_data());
    delay(PACKET_PERIOD_MS - POST_TRANSMIT_REST_MS);
  }
}
*/

byte rxDataBuffer[CCx_PACKT_LEN];
void WaitToReceiveStatusUntilTimeout(unsigned long timeout_ms) {
  if (radio_mode_ != RECEIVE_MODE) {
    SetRadioMode(RECEIVE_MODE);
  }
  unsigned long deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    // TODO(madadam): How many packets is the slave sending?  Maybe we need to
    // consume more than one to prevent RX buffer overflow?  Or we can shut
    // down the receiver after getting a valid packet.
    if (digitalRead(GDO0) == HIGH) {
      byte len;
      byte srcAddress, destAddress;
      byte rssi;
      byte lqi;
      int result = receiveData(rxDataBuffer, &len,
                               &srcAddress, &destAddress,
                               &rssi, &lqi);
      if (result == OK && len == sizeof(StatusMessage)) {
        Serial.println("Status:");
        // Got a status packet.
        // FIXME: Record it.
        //DebugPrintPacketTx(rxDataBuffer, sizeof(StatusMessage),
        //                   srcAddress, destAddress);
        StatusMessage status;
        status.ParseFromBuffer(rxDataBuffer);
        status.LogToSerial();
        break;
      } else {
        Serial.println("Bad packet"); // FIXME scaffold
      }
    }
  }
}

void MaybeReportStatus(unsigned long now) {
  if (IsTimerExpired(now, &last_status_report, STATUS_INTERVAL_MS)) {
    //memrep();
    Serial.print(now, DEC);
    Serial.print(" ");

    Serial.print(analogRead(SOLAR_PIN), DEC);
    Serial.print(" ");
    Serial.print(solar_state_, DEC);
    Serial.print(" ");
    Serial.print(day_cycle_.state(), DEC);

    Serial.println();
  }
}

void RunStartupSequence() {
  // Just to let the world know we're alive.
  for (int i = 0; i < 3; ++i) {
    analogWrite(BLUE_PIN, 255);
    delay(20);
    analogWrite(BLUE_PIN, 0);
    analogWrite(WHITE_PIN, 255);
    delay(20);
    analogWrite(WHITE_PIN, 0);
  }
}

inline void SetRadioMode(byte mode) {
  setRFBeeModeWith(mode);
  radio_mode_ = mode;
}

#define ADDRESS_CHECK 0  // 0: no check, 1: check, 2: check w/ broadcast (0)
void rfBeeInit(){
  CCx.PowerOnStartUp();
  byte config_id = 0;
  // Max power.
  byte config_pa_id = 7;
  CCx.Setup(config_id);
  CCx.Write(CCx_ADDR, RTS_ID);
  CCx.Write(CCx_PKTCTRL1, ADDRESS_CHECK | 0x04);
  CCx.setPA(config_id, config_pa_id);
  serialMode=SERIALDATAMODE;
  
  //GD00 is located on pin 2, which results in INT 0
  attachInterrupt(0, ISRVreceiveData, RISING);
  pinMode(GDO0, INPUT); // used for polling the RF received data

  // TCV mode is flaky.  Use TX-only mode.
  SetRadioMode(TRANSMIT_MODE);
}

// handle interrupt
void ISRVreceiveData(){
}

void DebugPrintPacketTx(byte* rxData, byte len,
                        byte srcAddress, byte destAddress) {
  DPrintByte("len", len);
  DPrintBufHex(" data", rxData, len);
  DPrintByte(" src", srcAddress);
  DPrintByte(" dst", destAddress);
  DPrintln();
}
