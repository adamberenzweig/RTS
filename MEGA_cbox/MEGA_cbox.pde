/******* Reflecting the Stars *********
Control Box.  Runs on the Arduino Mega.
Author: Adam Berenzweig
----------
**************************************/

char* versionblurb = "v.1.0 - Control Box"; 

#include <MessageTimer.h>
#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <PrintUtil.h>
#include "HardwareSerial.h"

// ----------------- Button Inputs -----------------
int asm_button_pin =  51;       // The pin from the Astronomy Button
int asl_button_pin = 52;        // The pin from the Astrology Button
int main_button_pin = 53;               // The pin from the Main Button
// ----------------- 

// Enums for naming some of the pins.  Note that this doesn't cover all of the
// mega's pins, just the ones we care about.
enum INPUT_PIN_NAMES {
  ASTRONOMY_BUTTON,
  ASTROLOGY_BUTTON,
  //MAIN_BUTTON,
  NUM_INPUT_PINS
};

// Keep this in sync with PIN_NAMES.
int input_pins[NUM_INPUT_PINS] = {
  24,  // ASTRONOMY_BUTTON (BUTTON_2_SIGNAL)
  22,  // ASTROLOGY_BUTTON (BUTTON_1_SIGNAL)
  //53,  // MAIN_BUTTON
};

enum OUTPUT_PIN_NAMES {
  BUTTON_1_LED,
  LED_STRING_1,
  BUTTON_2_LED,
  LED_STRING_2,
  BUTTON_3_LED,
  LED_STRING_3,
  
  ASTRONOMY_LED_0,
  ASTRONOMY_LED_1,
  ASTRONOMY_LED_2,
  ASTRONOMY_LED_3,
  ASTRONOMY_LED_4,
  ASTRONOMY_LED_5,
  ASTRONOMY_LED_6,
  ASTRONOMY_LED_7,
  ASTRONOMY_LED_8,
  ASTRONOMY_LED_9,
  ASTRONOMY_LED_10,
  ASTRONOMY_LED_11,

  ASTROLOGY_LED_0,
  ASTROLOGY_LED_1,
  ASTROLOGY_LED_2,
  ASTROLOGY_LED_3,
  ASTROLOGY_LED_4,
  ASTROLOGY_LED_5,
  ASTROLOGY_LED_6,
  ASTROLOGY_LED_7,
  ASTROLOGY_LED_8,
  ASTROLOGY_LED_9,
  ASTROLOGY_LED_10,
  ASTROLOGY_LED_11,

  NUM_OUTPUT_PINS
};

int output_pins[NUM_OUTPUT_PINS] = {
  23, 25, 27, 29, 26, 28,  // Button LEDs and LED strings
  30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, // Astronomy LEDs
  31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53,   // Astrology LEDs
};

char buf[64];
TimedMessage rts_messages[] = {
  { 60000, "TWK 215 60 0" },  // Sparse blue.
  { 60000, "TWK 215 60 1" },  // Sparse white.
  { 60000, "TWK 245 10 1" },  // Fast white.
  { 15000, "CST 100 0 100 2 4 6 8" },  // A constellation.
  { 1000,  "STATUS 2 3 4 5 6 7 8" },  // Get status from some stars.
};

// States for ACTIVE, SLEEPING, STANDBY.
// Each state has a TimedMessage array associated with it.  Pass that array to
// the MessageTimer?  Or to a higher level class that keeps the array, knows the
// length, which is the next message, etc.

// What happens in response to a button push?  Go to another state?
// Or keep a separate TimedMessage array for each state.  Now we need timed
// transitions between these states.  Is that overkill?

// State machine for button pushes and day cycle transitions.

// Button push:  LED fades etc. Each button has its own state machine.

// How would star wars work here? I don't think the MessageTimer class will
// support it.  Probably move the star wars code from the Master into here.


enum DAY_CYCLE_STATE {
  ACTIVE,
  SLEEPING,
  STANDBY,
};

MessageTimer message_timer_;

// FIXME: Use MessageTimer and its buffer.
//byte rtsMessageData[RTS_MESSAGE_SIZE];

// TODO(madadam): move to library?
void SetMessageFromString(char* input) {
  RtsMessage message(rtsMessageData);
  ParseRtsMessageFromString(input, &message);
}

void SetTestMessage(byte message_index) {
  // Make a copy because ParseRtsMessage is destructive.
  strcpy(buf, test_messages[message_index]);
  SetMessageFromString(buf);
}

// We can afford a larger buffer on the mega.  Make it big enough to handle
// error logging and debug packet printing from the master.
#define BUFFLEN 256
byte serialData[BUFFLEN];

void MaybeReadMasterSerial() {
  if (ReadMessageStringFromSerial(&Serial1, serialData, BUFFLEN)) {
    Serial.print("Read from master: ");
    Serial.println(serialData);
  }
}

void LedTestPattern() {
  for (int i = ASTRONOMY_LED_0; i < NUM_OUTPUT_PINS; ++i) {
    int pin = output_pins[i];
    DPrintByte("led pin", pin);
    DPrintln();
    digitalWrite(pin, HIGH);
    delay(600);
    digitalWrite(pin, LOW);
  }
}

void SwitchButtonLeds(byte value) {
  digitalWrite(output_pins[BUTTON_1_LED], value);
  digitalWrite(output_pins[BUTTON_2_LED], value);
  digitalWrite(output_pins[BUTTON_3_LED], value);
}

void setup() {
  for (int i = 0; i < NUM_INPUT_PINS; ++i) {
    int pin = input_pins[i];
    pinMode(pin, INPUT);
  }
  for (int i = 0; i < NUM_OUTPUT_PINS; ++i) {
    int pin = output_pins[i];
    pinMode(pin, OUTPUT);
  }
  Serial.begin(9600);
  Serial.println(versionblurb);

  // Master-Mega communication happens on Serial1.
  Serial1.begin(9600);

  LedTestPattern();
  SwitchButtonLeds(HIGH);
}

// FIXME: use states like the master test.
bool is_hibernating_ = false;

void loop() {
  LedTestPattern();  // FIXME for testing
  unsigned long now = millis();
  if (!is_hibernating_) {
    //HandleButtons();
    //UpdateLeds();
    //MaybeSendMessage(now)
  }

  MaybeReadMasterSerial();

  //HibernationControl(now);
} 
