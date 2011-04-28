/******* Reflecting the Stars *********
Control Box.  Runs on the Arduino Mega.
Author: Adam Berenzweig
----------
**************************************/

char* versionblurb = "v.1.0 - Control Box"; 

#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <PrintUtil.h>

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

byte rtsMessageData[RTS_MESSAGE_SIZE];

char buf[64];
char* test_messages[] = {
  "ALL_TWK 255 255 1",
  "SEL_TWK 100 255 0 2 3 4 5 6 7 8 9",
  "ALL_CST 50 50 50"
};

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
  LedTestPattern();
  SwitchButtonLeds(HIGH);
}

bool is_hibernating = false;

void loop() {
  LedTestPattern();  // FIXME for testing
  unsigned long now = millis();
  if (!is_hibernating) {
    //HandleButtons();
    //UpdateLeds();
    //MaybeSendMessage(now)
  } 
  
  //HibernationControl(now);
} 
