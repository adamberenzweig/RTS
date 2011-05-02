/******* Reflecting the Stars *********
RTS Puck code: RFBee slave, LED control, sleep control.
Version: 1.0
Authors: Adam Berenzweig, Richard Schwab, Corrie Van Sice, Icing Chang
Date: September 16, 2010
----------------------
**************************************/

//#define DEBUG

#ifdef MEMORY_DEBUG
#include <MemoryFree.h>
#endif
#include <PrintUtil.h>
#include <RtsMessage.h>
#include <RtsUtil.h>
#include <SleepControl.h>
#include <Twinkler.h>

// Puck-Specific Configuration
#define RTS_ID 10           // The Unique ID of this RFBee.
#define V_DIV_FACTOR 0.25180
// TODO(madadam): Where is this loss coming from?  I think it's the difference
// between the predicted voltage (by voltage divider math) at PDC3 and the
// actual voltage there.
#define FUDGE_FACTOR 1.051

/***************** Early Definitions ******************/
static char versionblurb[20] = "v.0.6 - SLAVE"; 
#define FIRMWAREVERSION 11 // 1.1, version number needs to fit in byte (0~255) to be able to store it into config
//#define FACTORY_SELFTEST
//#define INTERRUPT_RECEIVE
/****************************************************/

/***************** Includes ******************/
#include "debug.h"
#include "globals.h"
#include "Config.h"
#include "CCx.h"
#include "rfBeeSerial.h"

#ifdef FACTORY_SELFTEST
#include "TestIO.h"	// factory selftest
#endif
/*********************************************/


/***************** Global Definitions ******************/
#define GDO0 2 // used for polling the RF received data

// Pins
#define BLUE_PIN 5
#define WHITE_PIN 6
#define SOLAR_PIN 7
#define VOLTAGE_READ_PIN 3
int pin_number[NUM_LEDS];

// Radio
#define TRUSTED_SRC_ADDRESS 1
#define MAX_PACKET_LENGTH 32

// Power and Sleep
// Go to sleep if we don't receive anything in this interval:
#define LONELY_TIMEOUT_MS 2000UL
// When lonely, we sleep for an interval before waking up and turning the radio
// back on.  Tested up to 15 minutes.
// We use an exponential backoff:  starting from LONELY_SLEEP_SEC_MIN, double
// the sleep time each iteration (controlled by LONELY_SLEEP_EXPONENTIAL_FACTOR)
// until we hit LONELY_SLEEP_SEC_MAX.  Reset to the MIN time whenever we recieve
// a good message.
#define LONELY_SLEEP_SEC_MIN 16
#define LONELY_SLEEP_SEC_MAX 15 * 60  // 15 min
#define LONELY_SLEEP_EXPONENTIAL_FACTOR 2

// RadioSleep settings:
// When sleeping the radio, wake it up with this much time until we anticipate
// receiving the next packet:
#define RADIO_WAKE_LEEWAY_MS 100
// How long to sleep after successfully receiving a packet.  Sleep almost
// until the next message period, with a bit of leeway.
// Set to 0 to disable radio sleeping.
unsigned long kRadioSleepTimeMs = MESSAGE_PERIOD_MS - RADIO_WAKE_LEEWAY_MS;

// Voltage thresholder settings.
#define VOLTAGE_THRESHOLD_WINDOW_SEC 30
#define VOLTAGE_THRESHOLD_LOW 3.77
#define VOLTAGE_THRESHOLD_HIGH 3.85

#define VOLTAGE_WINDOW_LEN 10
#define LOW_VOLTAGE_SLEEP_TIME_SEC 3600  // One hour
SmoothedThreshold voltage_threshold_;

// Status/Debug
#define STATUS_INTERVAL_MS 2000UL
const int debug_level = 0;
unsigned int num_rx_since_status = 0;
unsigned int num_bad_rx = 0;
byte last_state = 0;  // Only for status reporting.
unsigned long total_sleep_time = 0;

// Timers:
unsigned long last_status_report = 0;
// Time since we started radio sleep:
unsigned long last_radio_sleep_start = 0;
// The sleep-when-lonely timer.
unsigned long last_rx_ms = 0;
unsigned int next_lonely_sleep_interval = LONELY_SLEEP_SEC_MIN;
// led control timer
unsigned long last_led_control_time = 0;
// Timeout for attention mode.
unsigned long last_attention_time = 0;
// Attention mode timeout doesn't need to be very long because if the control
// box really wants us to be at attention, it will be sending attention commands
// pretty frequently (every 100ms at least).
#define ATTENTION_TIMEOUT_MS 2000

// State variables:

// See the comment about kRadioSleepTimeMs.
bool is_radio_sleeping = false;
// When at_attention, never sleep the radio.
bool at_attention = false;
byte current_msg_checksum = 0;
// The currently active twinkler, or NULL if not twinkling.
Twinkler* twinkler = NULL;
// For obeying SLEEP_NOW commands.
unsigned int sleep_on_next_loop_for_sec = 0;

/*********************************************/

// This is lame. Arduino doesn't support 'new' and 'delete', so I can't
// Have a true factory method that dynamically creates the object of the right
// type.  Instead, keep an object of each type around, and re-initialize it
// in the factory method.
RandomTwinkler random_twinkler;
ConstellationTwinkler constellation_twinkler;

Twinkler* TwinklerFactory(byte led_state, const RtsMessage& message) {
  if (led_state == LED_CONSTELLATION) {
    constellation_twinkler.Init(message);
    return &constellation_twinkler;
  } else if (led_state == LED_TWINKLE) {
    random_twinkler.Init(message);
    return &random_twinkler;
  } else if (led_state == LED_OFF) {
    TurnOffLightsNice();
    return NULL;
  }
  
  // No message, default params.
  random_twinkler.Init();
  return &random_twinkler;
}

//===================this is for slave RFBee==================

void setup(){
  // For the voltage reading pin. NOTE: requires cutting VREF from V+.
  analogReference(INTERNAL);

  randomSeed(analogRead(0));

  //do extra initalization
  Config.set(CONFIG_MY_ADDR,RTS_ID);        //modify the number to specify an unique address for RFBee itself 
  setMyAddress();
  Config.set(CONFIG_ADDR_CHECK, 0);  // 0: no check, 1: check, 2: check w/ broadcast (0)
  setAddressCheck();
  //==========================

  if (Config.initialized() != OK) {
    Serial.begin(9600);
    Config.reset();
  }

  setUartBaudRate();
  delay(2);

  Serial.println(versionblurb);
  Serial.print("ID: ");
  Serial.println(RTS_ID, DEC);
  
  InitVoltageThresholder();
  InitLedStates();
  InitSleepControl();
  
#ifndef PRODUCTION  
  RunStartupSequence();
#endif

  // Turn the radio on last, otherwise we can get RX buffer overflows
  // if the radio is on but we're not consuming the data.
  rfBeeInit();
}

void InitVoltageThresholder() {
  voltage_threshold_.Init(VOLTAGE_THRESHOLD_WINDOW_SEC,
                          VOLTAGE_THRESHOLD_LOW,
                          VOLTAGE_THRESHOLD_HIGH);
}

void RunStartupSequence() {
  // Just to let the world know we're alive.
  for (int i = 0; i < 4; ++i) {
    if (i > 0) {
      delay(100);
    }
    analogWrite(BLUE_PIN, 100);
    delay(50);
    analogWrite(BLUE_PIN, 0);
    analogWrite(WHITE_PIN, 100);
    delay(50);
    analogWrite(WHITE_PIN, 0);
  }
}

void InitLedStates() {
  pin_number[BLUE] = BLUE_PIN;
  pin_number[WHITE] = WHITE_PIN;
  last_led_control_time = millis();
}

// TODO: move some of this to RtsRadio?
void rfBeeInit(){
  CCx.PowerOnStartUp();
  setCCxConfig();
  serialMode=SERIALDATAMODE;
  // GD00 is located on pin 2, which results in INT 0:
  attachInterrupt(0, ISRVreceiveData, RISING);
  pinMode(GDO0,INPUT); // used for polling the RF received data
  setRFBeeModeWith(RECEIVE_MODE);
}

// handle interrupt
void ISRVreceiveData(){
}

void loop(){
  unsigned long now = millis();
  
  // This needs to be called before MaybeRxMessage in this loop.
  MaybeSleep(now);
  // Recompute now, we might have slept a while.
  now = millis();

  MaybeRxMessage(now);
  
  MaybeRunLedControl(now);
  
  MaybeReportStatus(now);
  
  // TODO(madadam): Is there any good reason to do this anymore?  Maybe starwars
  // responsiveness will be faster if I don't.
  delay(30);  // FIXME: FADE_INTERVAL_MS
}

bool HandleSpecialCommand(byte command, const RtsMessage& message) {
  if (command == ATTENTION) {
    at_attention = true;
    last_attention_time = millis();
    return true;
  }
  if (command == AT_EASE) {
    at_attention = false;
    return true;
  }
  if (command == SLEEP_NOW) {
    unsigned int sleep_time_sec = message.getParam(SLEEP_TIME_SEC);
    byte multiplier = message.getParam(SLEEP_TIME_MULTIPLIER);
    if (multiplier) {
      sleep_time_sec *= multiplier;
    }
    // Sleep on the next loop.  Otherwise returning from here is awkward.
    sleep_on_next_loop_for_sec = sleep_time_sec;
    return true;
  }
  // TODO(madadam): Handle ALL_RESET. Does voltage threshold change?
  return false;
}

static byte rxData[CCx_PACKT_LEN];

void MaybeRxMessage(unsigned long now) {
  byte did_rx_good_packet = 0;
  byte new_state = IGNORE;

  // Check if we need to time out of attention mode.
  if (at_attention &&
      IsTimerExpired(now, &last_attention_time, ATTENTION_TIMEOUT_MS)) {
    at_attention = 0;
  }

  // Check if the radio needs to be woken up.
  if (is_radio_sleeping &&
      IsTimerExpired(now, &last_radio_sleep_start, kRadioSleepTimeMs)) {
    // DPrintln("radio sleep off");
    setRFBeeModeWith(RECEIVE_MODE);
    is_radio_sleeping = 0;
    // Reset the lonely-sleep timer, we can't have heard anything while
    // the radio was sleeping.
    last_rx_ms = now;
    // Give the radio some time to wake up:
    delayMicroseconds(200);
  }
  
  // Decode all the packets available, to prevent RX buffer overflow.
  while ( digitalRead(GDO0) == HIGH ) {
    if (debug_level > 0) {
      DPrint("Rx => ");
    }
    last_rx_ms = millis();
    // We heard something, so reset the lonely sleep exponential backoff.
    next_lonely_sleep_interval = LONELY_SLEEP_SEC_MIN;

    bool is_good_packet = waitAndReceiveRFBeeData(rxData);
    if (is_good_packet) {
      did_rx_good_packet = 1;
      num_rx_since_status++;
      RtsMessage message(rxData);
      if (debug_level > 0) {
        message.DebugPrint();
      }
      byte this_packet_command = message.getMyState(RTS_ID);
      if (!HandleSpecialCommand(this_packet_command, message)) {
        // Special commands aren't LED states, so don't set new_state for them.
        if (this_packet_command != IGNORE) {
          new_state = this_packet_command;
        }
      } else {
        new_state = this_packet_command;
      }
    }
  }

  if (did_rx_good_packet && kRadioSleepTimeMs && !at_attention) {
    last_radio_sleep_start = last_rx_ms;
    setRFBeeModeWith(SLEEP_MODE);
    is_radio_sleeping = 1;
    // DPrintln("radio sleep on");
  }
  
  // Only act on the most recent packet, and only if it differs
  // from the current state (detected by a change in the checksum).
  if (new_state != IGNORE) {
    last_state  = new_state;
    RtsMessage message(rxData);
    byte checksum = message.checksum();
    // Note that we're using a braindead checksum and it's only one byte.
    // We might get collisions, in which case the puck will ignore the
    // new message.
    if (checksum != current_msg_checksum) {
      twinkler = TwinklerFactory(new_state, message);
      if (twinkler && debug_level > 0) {
        DPrintln(twinkler->Name());
      }
      current_msg_checksum = checksum;
    }
  }
}

bool ValidatePacket(byte* rxData, byte len,
                    byte srcAddress, byte destAddress, byte rssi) {
  if (len > MAX_PACKET_LENGTH) {
    return 0;
  }
  if (srcAddress != TRUSTED_SRC_ADDRESS) {
    return 0;
  }
  return 1;
}

byte waitAndReceiveRFBeeData(byte* rxData)
{
  byte len;
  byte srcAddress;
  byte destAddress;
  byte rssi;  // signal strength indicator
  byte lqi;   // quality indicator
  int result;
  byte good_packet = 1;

  result = receiveData(rxData, &len, &srcAddress, &destAddress, &rssi , &lqi);
  
  if (result == ERR) {
    writeSerialError();
    good_packet = 0;
  } else if (result == NOTHING) {
    good_packet = 0;
  }

  if (good_packet) {
    if (!ValidatePacket(rxData, len, srcAddress, destAddress, rssi)) {
      num_bad_rx++;
      DPrintln("Rejected packet.");
      good_packet = 0;
    }
  }
  
  if (len < RTS_MESSAGE_SIZE) {
    num_bad_rx++;
    DPrintln("packet too short.");
    good_packet = 0;
  }

  if (!good_packet) {
    DebugPrintPacket(result, rxData, len, srcAddress, destAddress, rssi, lqi);
  }
  
  return good_packet;
}

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

inline float ScaleVoltage(int raw_value) {
  // Reference voltage.  Atmel docs say it's 1.1, but we don't have a capacitor
  // and empirically using 1.05 seems closer.
  #define VREF 1.05
  return float(raw_value) * FUDGE_FACTOR * VREF / (1023.0 * V_DIV_FACTOR);
}

void MaybeReportStatus(unsigned long now) {
  if (IsTimerExpired(now, &last_status_report, STATUS_INTERVAL_MS)) {
    // DPrintln("\n---Status---");
#ifdef MEMORY_DEBUG
    memrep();
#endif
    //DPrintUL("num_rx", num_rx_since_status);
    
    // TODO(madadam): Maybe the voltage reading should be in its own function.
    // But currently we want it at the same interval as status updates,
    // so it's fine here.
    int voltage_reading = analogRead(VOLTAGE_READ_PIN);
    float scaled_voltage = ScaleVoltage(voltage_reading);
    voltage_threshold_.Update(scaled_voltage, now);
    /*
    DPrintInt(" pc3", voltage_reading);
    // DPrintFloat(" v", voltage_threshold_.smoothed_value());
    DPrintByte("", voltage_threshold_.state());
    
    DPrintln();
    if (at_attention) {
      DPrintln("At ATTN");
    }
    */

    int solar_reading = analogRead(SOLAR_PIN);

    // Logging.
    Serial.print(now, DEC);
    Serial.print(" ");
    Serial.print(total_sleep_time, DEC);
    Serial.print(" ");
    Serial.print(voltage_reading, DEC);
    Serial.print(" ");
    Serial.print(analogRead(SOLAR_PIN), DEC);
    Serial.print(" ");
    Serial.print(last_state, DEC);
    Serial.print(" ");
    Serial.print(voltage_threshold_.smoothed_value());
    Serial.println();

    num_rx_since_status = 0;
    num_bad_rx = 0;
  }
}

void MaybeRunLedControl(unsigned long now) {
  if (IsTimerExpired(now, &last_led_control_time, FADE_INTERVAL_MS)) {
    LedControlTimeSlice();
  }
}

// ledControlTimeSlice is called every N milliseconds.  It uses the currently
// active Twinkler to control the LED values.
void LedControlTimeSlice() {
  if (NULL == twinkler) {
    return;
  }
  for (int led = 0; led < NUM_LEDS; ++led) {
    if (twinkler->shouldWrite(led)) {
      analogWrite(pin_number[led], twinkler->Value(led));
    }
  }
  twinkler->Update();
  //twinkler->DebugPrint();
}

void FullSleepFor(unsigned int sleep_time_sec) {
  DPrintInt("\nSleeping (s)", sleep_time_sec);
  DPrintln();
  delay(1);  // Allow the buffer to flush.  FIXME Remove in prod?

  // TODO: Make a Twinkler mode for shutting down (fade-out).
  // How to know when it's finished so that we can go to sleep?
  // Could just count the cycles and then shut down.
  // FIXME: Add a FallingAsleepTwinkler?
  
  // FIXME: we haven't actually turned off the lights yet.
  // we should go into a "falling_asleep" mode until we finish
  // turning off the lights nicely.  What happens if we rx something during
  // that time?
  TurnOffLightsNice();
  
  // Go to sleep.  This call won't return until we're woken up.
  unsigned long sleep_time = 1000UL * sleep_time_sec;
  setRFBeeModeWith(SLEEP_MODE);
  total_sleep_time += sleepWithTimeout(sleep_time);

  // Just for debugging. don't do this in prod.
  // Do this before turning the radio back on, otherwise we can get
  // RX buffer overflows because the radio might receive messages while
  // we're in here.
  // RunStartupSequence();
  
  setRFBeeModeWith(RECEIVE_MODE);
  is_radio_sleeping = 0;
  // Reset the sleep-if-lonely clock.
  last_rx_ms = millis();

  // No twinkler until we get a message.
  twinkler = NULL;
}

void MaybeSleep(unsigned long now) {
  if (sleep_on_next_loop_for_sec > 0) {
    // Note that when we wake from this we flush the RX buffer, otherwise we
    // might sleep multiple times in a row if more than one packet was sent.
    FullSleepFor(sleep_on_next_loop_for_sec);
    sleep_on_next_loop_for_sec = 0;
    return;
  }

  // Low-voltage sleep.
  if (voltage_threshold_.state() == SmoothedThreshold::STATE_LOW) {
    DPrintln("\nLO V");
    FullSleepFor(LOW_VOLTAGE_SLEEP_TIME_SEC);
    return;
  }

  // Lonely Timer - for when the Rx no longer hears from the Tx, it will go to
  // sleep.
  if (!is_radio_sleeping &&
      IsTimerExpired(now, &last_rx_ms, LONELY_TIMEOUT_MS)) {
    DPrintln("\nLonely");
    FullSleepFor(next_lonely_sleep_interval);
    next_lonely_sleep_interval *= LONELY_SLEEP_EXPONENTIAL_FACTOR;
    if (next_lonely_sleep_interval > LONELY_SLEEP_SEC_MAX) {
      next_lonely_sleep_interval = LONELY_SLEEP_SEC_MAX;
    }
  }
}

void TurnOffLightsNice() {
 // Ok, not so nice.  :)
 analogWrite(WHITE_PIN, 0);
 analogWrite(BLUE_PIN, 0);
}
