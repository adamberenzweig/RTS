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

char versionblurb[20] = "v.1.0 - MASTER";

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
// Last time we changed the message
unsigned long last_message_time = 0;
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
#define MESSAGE_MODE TEST_CYCLE

// FIXME:  Getting short on SRAM again because of all the message
// strings.  Put them in progmem.

// Test mode parameters:
#define NUM_MSG_TO_SEND -1   // -1 to go forever

struct TestMessage {
  unsigned long duration_ms;
  char* message;
};

// Messages to cycle through in test mode 1.
TestMessage test_messages[] = {
//  { 240000, "ALL_TWK 215 60 0"},  // Sparse blue twinkle.
//  { 240000, "ALL_TWK 215 60 1"},  // Sparse white twinkle.
//  { 0,      "SW 20 1 0 2 16"},     // Star wars both ways.
//  { 240000, "ALL_TWK 245 10 1"},  // fast white twinkle

  { 10000, "ALL_TWK 245 10 1"},  // fast white twinkle
  { 1000, "STATUS 13"},  // Tell Puck 13 to report status.

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

// Big enough for the longest test_message.
char buf[50];
// The index of the current test message.
byte current_message = 0;
#define TESTING_MESSAGE_PERIOD_MS 10000UL
unsigned long message_period_ms = TESTING_MESSAGE_PERIOD_MS;
// If nonzero, check the solar voltage once per interval, possibly transitioning
// to daytime sleep mode.
#define SOLAR_SLEEP_CHECK_INTERVAL_MS 5000
#define TESTING_SLEEP_TIME_SEC 1200  // 60*20 20mins

// TODO(madadam): PROGMEM.
char* all_attention_message = "ATTN";
char* at_ease_message = "ATEZ";
char* all_off_message = "ALL_OFF";
char* star_wars_prefix = "SEL_CST 0 0 255";
char* sleep_one_hour_message = "SLEEP 60 60";

// Work around a bug that causes the RFBee to stop transmitting after a while.
// Resetting the rfbee radio mode periodically seems to help.
#define RADIO_RESET_INTERVAL_PACKETS 1000UL

// FIXME: need to double-buffer this to swap current good message while parsing the next one.
byte rtsMessageData[RTS_MESSAGE_SIZE];

//================this is for Master RFBee======================

// TODO(madadam): I think the EEPROM isnt being used right throughout this code.
// Why set it from a #define and then just read it back again?  Only need to
// use it if we can't hardcode the config data and instead want to configure
// with a serial port or similar.

void SetMessageFromString(char* input) {
  RtsMessage message(rtsMessageData);
  ParseRtsMessageFromString(input, &message);
}

void SetMessage(const char* message) {
  // Make a copy because ParseRtsMessage is destructive.
  strcpy(buf, message);
  SetMessageFromString(buf);
}

void SetStarWarsMessage(byte* ids, int num_ids) {
  FormatStarWarsMessage(buf, star_wars_prefix, ids, num_ids);
  SetMessageFromString(buf);
}

void SetTestMessage(byte message_index) {
  message_period_ms = test_messages[message_index].duration_ms;
  SetMessage(test_messages[message_index].message);
}

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
  current_message = 0;
  last_message_time = millis();
  SetTestMessage(current_message);
}

// FIXME: bug probably here.  either:
// 1) since we don't use smoothedthreshold, noise in the solar reading leads to
// accidental transition from night->day.
// 2) standby timer expires without solar transition.  we go active, but
// solar_state_ is still DAY.  then what happens?
//  ACTIVE, DAY
//  ACTIVE, NIGHT (call InitActiveCycle() again, resets but harmless.)
//  SLEEPING, NIGHT
//  SLEEPING, DAY
//  STANDBY, DAY
//  ACTIVE, NIGHT (solar triggered)
// 3) power cycled during the day.  solar_state starts as NIGHT. what happens?

void CheckForDayCycleTransition(unsigned long now) {
  if ((STANDBY_DURATION_MS > 0 &&
       day_cycle_state_ == STANDBY &&
       IsTimerExpired(now, &last_cycle_transition, STANDBY_DURATION_MS)) ||
      (SolarTransition(now, &solar_state_) && solar_state_ == NIGHT)) {
    // Sun just set.  Wake up, little vampire.
    day_cycle_state_ = ACTIVE;
    InitActiveCycle();
  }
  // FIXME should these be else-ifs?  would we ever transition twice?
  if (day_cycle_state_ == ACTIVE &&
      IsTimerExpired(now, &last_cycle_transition, ACTIVE_DURATION_MS)) {
    day_cycle_state_ = SLEEPING;
    // Bedtime Sequence:
    SetMessage(all_off_message);
    SendNMessages(16);
    SetMessage(sleep_one_hour_message);  // 60*60 = one hour.
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
        MaybeChangeMessage(now);
      } else if (MESSAGE_MODE == MESSAGE_SERIAL) {
        TryReadMessageFromSerial();
      }
    }
    if (day_cycle_state_ != STANDBY) {
      if (!MaybeRunStarWars()) {
        MaybeSendMessage(now, PACKET_PERIOD_MS);
      }
    }
  }
  MaybeReportStatus(now);
}

void SetNextMessage() {
  int num_msgs = sizeof(test_messages) / sizeof(TestMessage);
  current_message = (current_message + 1) % num_msgs;
  SetTestMessage(current_message);
}

// Return true if we ran a star wars sequence.
bool MaybeRunStarWars() {
  // Assume that the message has already been written.
  RtsMessage message(rtsMessageData);
  // We can't keep the message around while star wars is running because the
  // underlying rtsMessageData buffer gets reused by each message in the
  // sequence.  So extract all params now.  If that becomes a pain, then make a
  // separate buffer for the star wars message and pass the full message to
  // RunStarWars().
  if (message.command() != STAR_WARS) {
    return false;
  }
  byte num = message.getParam(STAR_WARS_NUM);
  byte type = message.getParam(STAR_WARS_TYPE);
  byte min_id = message.getId(0);
  byte max_id = message.getId(1);
  RunStarWars(num, type, min_id, max_id);
  // After we're done, transition to the next message.
  SetNextMessage();
  last_message_time = millis();
  return true;
}

void RunStarWars(byte num, byte type, byte min_puck, byte max_puck) {
  DPrintln("Running Star Wars.");
  SetMessage(all_attention_message);
  SendNMessages(16);
  // Things go bad if this is less than 60ms.  Maybe speeding the slave up by
  // taking out debug printing could handle it.
  const int inter_msg_interval = 75;
#define kNumPucksPerMessage 2

  byte ids[kNumPucksPerMessage];
  for (int j = 0; j < num; ++j) {
    for (int i = min_puck; i <= max_puck; ++i) {
      // Sending ATTENTION in between every message gives a pretty good
      // chance that all pucks will get it.  If they don't they'll go into
      // radio sleep each time they receive a packet.
      SetMessage(all_attention_message);
      SendOneMessage();
      delay(inter_msg_interval);

      ids[0] = i;
      if (type == 2) {
        // both ways
        ids[1] = max_puck - i + min_puck;
      }
      SetStarWarsMessage(ids, kNumPucksPerMessage);
      SendOneMessage();
      delay(inter_msg_interval);
    }
    // Now pause for a bit, but keep sending messages so pucks don't sleep.
    SetMessage(all_off_message);
    SendNMessages(4);
  }
  // A bit dangerous, if somebody misses this, they'll run out of batteries
  // real fast.  We send regular AT_EASE messages to prevent that.
  SetMessage(at_ease_message);
  SendNMessages(10);
}

void MaybeChangeMessage(unsigned long now) {
  if (IsTimerExpired(now, &last_message_time, message_period_ms)) {
    // Since we've introduced the ATTENTION command, I'm worried that pucks
    // could get stuck at attention if they miss the AT_EASE at the end of the
    // star wars routine.  To be safe, every time we change messages, send
    // AT_EASE.  Pucks already at ease will either miss it, or not care, and
    // pucks stuck at attention have a chance to recover.
    // TODO(madadam): Do something similar in the Mega for prod.
    SetMessage(at_ease_message);
    SendOneMessage();
    // TODO(madadam): Brittle; bug lurking here if message period gets shorter.
    delay(100);  // Not a full packet period.

    SetNextMessage();
  }
}

void TryReadMessageFromSerial() {
  RtsMessage message(rtsMessageData);
  if (ReadRtsMessageFromSerial(serialData, BUFFLEN, &message)) {
    DPrintInt("Read cmd", message.command());
    DPrintln();
  }
}

void MaybeSendMessage(unsigned long now, int period_ms) {
  if ((NUM_MSG_TO_SEND < 0 || tx_counter < NUM_MSG_TO_SEND) &&
       IsTimerExpired(now, &last_tx_time, period_ms)) {
    SendOneMessage();
    
    // FIXME: In prod, attached to the Mega, there won't be any LEDs, right?
    // Remove this so we don't confuse the mega by writing to pins. 
    DFlashLeds(WHITE_PIN);
  }
}

void SendOneMessage() {
  transmitData(rtsMessageData, RTS_MESSAGE_SIZE, srcAddress, destAddress);
  // NOTE: This slows things down a lot!  If you want fast performance for
  // Star Wars mode, don't do this here:
  //DebugPrintPacketTx(rtsMessageData, RTS_MESSAGE_SIZE, srcAddress, destAddress);
  tx_counter++;
  if (RADIO_RESET_INTERVAL_PACKETS > 0 &&
      tx_counter % RADIO_RESET_INTERVAL_PACKETS == 0) {
    setRFBeeModeWith(TRANSMIT_MODE);
  }
}

void SendNMessages(int n) {
  for (int i = 0; i < n; ++i) {
    SendOneMessage();
    delay(PACKET_PERIOD_MS);
  }
}

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

// TODO(madadam): Move to a library.  SleepControl?
int IsTimerExpired(unsigned long now,
                   unsigned long* last_time,
                   unsigned long interval_ms) {
  if (now < *last_time) {
    // time overflow, happens every 50 days
    *last_time = 0;
  }
  // Don't underflow the subtraction.
  if (now >= interval_ms &&
      *last_time < now - interval_ms) {
    *last_time = now;
    return 1;
  }
  return 0;
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

void rfBeeInit(){
  DEBUGPRINT()
  
  CCx.PowerOnStartUp();
  setCCxConfig();
  // TCV mode is flaky.  Use TX-only mode.
  setRFBeeModeWith(TRANSMIT_MODE);
 
  serialMode=SERIALDATAMODE;
  
  //GD00 is located on pin 2, which results in INT 0
  attachInterrupt(0, ISRVreceiveData, RISING);
  pinMode(GDO0, INPUT); // used for polling the RF received data
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
