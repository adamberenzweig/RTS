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
#include "Config.h"
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
// Last time we checked the solar voltage for solar-sleep.
unsigned long last_solar_check = 0;
unsigned long last_cycle_transition = 0;

// Histeresis thresholds for the solar sleep transitions.  Go into DAY mode when
// the solar reading goes above SOLAR_THRESHOLD_HIGH, and go into NIGHT mode
// when it drops below SOLAR_THRESHOLD_LOW.
#define SOLAR_THRESHOLD_LOW 20
#define SOLAR_THRESHOLD_HIGH 100

enum SOLAR_STATE {
  DAY,
  NIGHT
};
byte solar_state_ = NIGHT;

enum DAY_CYCLE_STATE {
  ACTIVE,    // Night-time active duty. Let there be light.
  SLEEPING,  // After active duty, put the slaves to bed to save power.
  STANDBY,   // Before sunset, no more sleep messages, let the slaves get ready.
};
byte day_cycle_state_ = ACTIVE;

#define ACTIVE_DURATION_MS 14400000UL    // 4 hours
#define SLEEPING_DURATION_MS 64800000UL // 18 hours
// If this is > 0, then only standby for the specified time.  Otherwise, standby
// indefinitely until the SOLAR_STATE transitions back to night.
#define STANDBY_DURATION_MS 7200000UL // 2 hours

// Voltage thresholder settings.
#define VOLTAGE_THRESHOLD_FRACTION .85
#define VOLTAGE_HISTERESIS_FRACTION .05
#define VOLTAGE_WINDOW_LEN 10

// MESSAGE_MODE controls how we get messages before sending them out.
enum MESSAGE_MODES {
  // Listen for raw message bytes on the incoming serial port, and transmit
  // once 32 bytes have been received.
  RAW_SERIAL,

  // Cycle through the test_messages.
  TEST_CYCLE,

  // Read messages from the serial port.
  MESSAGE_SERIAL,
};
#define MESSAGE_MODE MESSAGE_SERIAL

// FIXME:  Getting short on SRAM again because of all the message
// strings.  Put them in progmem.

// Test mode parameters:
#define NUM_MSG_TO_SEND -1   // -1 to go forever

MessageTimer message_timer_;

// Messages to cycle through in test mode 1.
TimedMessage test_messages[] = {
//  { 240000, "TWK 215 60 0"},  // Sparse blue twinkle.
//  { 240000, "TWK 215 60 1"},  // Sparse white twinkle.
//  { 0,      "SW 20 1 0 2 16"},     // Star wars both ways.
//  { 240000, "TWK 245 10 1"},  // fast white twinkle

  { 60000, "TWK 215 60 0"},  // Sparse blue twinkle.
  /* I think we're busting RAM with all this.
  { 60000, "TWK 215 60 1"},  // Sparse white twinkle.
  { 60000, "TWK 245 10 1"},  // fast white twinkle
  { 1000,  "STATUS 68"},  // Tell each puck to report status.
  { 1000,  "STATUS 69"},
  { 1000,  "STATUS 70"},
  { 1000,  "STATUS 71"},
  { 1000,  "STATUS 72"},
  { 1000,  "STATUS 73"},

  { 1000,  "STATUS 75"},
  { 1000,  "STATUS 76"},
  { 1000,  "STATUS 78"},
  { 1000,  "STATUS 79"},
  { 1000,  "STATUS 86"},
  { 1000,  "STATUS 87"},
  { 1000,  "STATUS 88"},
  { 1000,  "STATUS 89"},
  */

  /*
  { 10000, "TWK 245 10 1"},  // fast white twinkle
  { 10000, "STATUS 13"},  // Tell Puck 13 to report status.
  */

  // "ALL_CST 100 0 100", // All constellation white w/ medium fade-in
/*  
  "ALL_TWK 255 255 1",         // fast white
  "ALL_TWK 20 255 0",          // slow blue
  "ALL_TWK 20 255 0",          // slow blue
  "ALL_TWK 20 255 0",          // slow blue
  "SEL_CST 100 0 100 2 4 6 8", // Every other one constellation white
  "ALL_OFF",                   // 10 second break
*/
};


// The index of the current test message.
byte current_message_ = 0;  // FIXME moved inside MessageTimer?
// Current state of the radio.
byte radio_mode_ = TRANSMIT_MODE;

// If nonzero, check the solar voltage once per interval, possibly transitioning
// to daytime sleep mode.
#define SOLAR_SLEEP_CHECK_INTERVAL_MS 5000
#define TESTING_SLEEP_TIME_SEC 1200  // 60*20 20mins

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
  
  Config.set(CONFIG_MY_ADDR, RTS_ID);
  setMyAddress();
  //==========================

  if (RTS_MESSAGE_SIZE > CCx_PACKT_LEN) {
    DPrintln("Error: RTS_MESSAGE_SIZE too long.");
  }
  
  if (Config.initialized() != OK) 
  {
    Serial.begin(9600);
    Serial.println("Initializing config"); 
    Config.reset();
  }
  setUartBaudRate();

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

TimedMessage bedtime_sequence[] =  {
  { 4000, "OFF" },
  { 0,    "SLEEP 60 60", },  // Sleep one hour, indefnitely.
};

// TODO(madadam): In production, this code or its equivalent will run in the
// Mega.  Rip it out of here, or at least refactor to a library.
void CheckForDayCycleTransition(unsigned long now) {
  if ((STANDBY_DURATION_MS > 0 &&
       day_cycle_state_ == STANDBY &&
       IsTimerExpired(now, &last_cycle_transition, STANDBY_DURATION_MS)) ||
      (SolarTransition(now, &solar_state_) && solar_state_ == NIGHT)) {
    // Sun just set.  Wake up, little vampire.
    day_cycle_state_ = ACTIVE;
    InitActiveCycle();
  }
  if (day_cycle_state_ == ACTIVE &&
      IsTimerExpired(now, &last_cycle_transition, ACTIVE_DURATION_MS)) {
    day_cycle_state_ = SLEEPING;
    // Bedtime Sequence:
    byte num_msgs = (byte)(sizeof(bedtime_sequence) / sizeof(TimedMessage));
    message_timer_.StartWithMessages(bedtime_sequence, num_msgs);
  }
  if (day_cycle_state_ == SLEEPING &&
      IsTimerExpired(now, &last_cycle_transition, SLEEPING_DURATION_MS)) {
    day_cycle_state_ = STANDBY;
    // In standby, we don't transmit, but set this for safety.
    SetMessage(all_off_message);
  }
}

void loop(){
  unsigned long now = millis();

  // Only use the day/night cycles in test_messages mode.
  if (MESSAGE_MODE == TEST_CYCLE) {
    CheckForDayCycleTransition(now);
  }

  if (MESSAGE_MODE == RAW_SERIAL) {
    readSerialData();
  } else {
    if (day_cycle_state_ == ACTIVE) {
      if (MESSAGE_MODE == TEST_CYCLE) {
        message_timer_.MaybeChangeMessage(now);
      } else if (MESSAGE_MODE == MESSAGE_SERIAL) {
        TryReadMessageFromSerial();
      }
    }
    if (day_cycle_state_ != STANDBY) {
      MaybeSendMessage(now, PACKET_PERIOD_MS);
    }
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

// FIXME: change to read timed message from serial?  or also have another
// option, message with no timeout, and have this be a special case.
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
    
    // FIXME: In prod, attached to the Mega, there won't be any LEDs, right?
    // Remove this so we don't confuse the mega by writing to pins. 
    DFlashLeds(WHITE_PIN);
  }
}

void SendOneMessage(byte* message_data) {
  if (radio_mode_ != TRANSMIT_MODE ||
      (RADIO_RESET_INTERVAL_PACKETS > 0 &&
       tx_counter % RADIO_RESET_INTERVAL_PACKETS == 0)) {
    SetRadioMode(TRANSMIT_MODE);
  }
  // FIXME: message_timer_.message_data()
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

// TODO(madadam): Use SmoothedThreshold here.
bool SolarTransition(unsigned long now, byte* solar_state) {
  if (SOLAR_SLEEP_CHECK_INTERVAL_MS > 0 &&
      IsTimerExpired(now, &last_solar_check, SOLAR_SLEEP_CHECK_INTERVAL_MS)) {
    int solar_reading = analogRead(SOLAR_PIN);
    if (solar_reading < SOLAR_THRESHOLD_LOW && *solar_state == DAY) {
      *solar_state = NIGHT;
      return true;
    } else if (solar_reading > SOLAR_THRESHOLD_HIGH &&
               *solar_state == NIGHT) {
      *solar_state = DAY;
      return true;
    }
  }
  return false;
}

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
    Serial.print(day_cycle_state_, DEC);

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
  radio_mode_ = mode;
  setRFBeeModeWith(mode);
}

void rfBeeInit(){
  DEBUGPRINT()
  
  CCx.PowerOnStartUp();
  setCCxConfig();
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
