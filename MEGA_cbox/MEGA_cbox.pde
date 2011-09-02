/******* Reflecting the Stars *********
Control Box.  Runs on the Arduino Mega.
Author: Adam Berenzweig
----------
**************************************/

char versionblurb[] = "v.1.0 - Control Box"; 
// Or try:
// Serial.println(F("v.1.0 - Control Box"));

#include <DayCycle.h>
#include "HardwareSerial.h"
#include <DateTime.h>
#include <Flash.h>
#include <MessageTimer.h>
#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <PrintUtil.h>
#include <RTClib.h>
#include <RtsUtil.h>
#include <SD.h>
#include <Twinkler.h>
#include <Wire.h>

// For status reporting:
#define MIN_SLAVE_ID 2
#define MAX_SLAVE_ID 201

#define NUM_BUTTONS 12

int button_signal_pins[NUM_BUTTONS] = {
  //23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45
  // Permuted due to random wiring:
  22, 45, 41, 31, 29, 27, 37, 39, 25, 35, 33, 43
};

int button_led_pins[NUM_BUTTONS] = {
  // PWM pins 2-13.  Permuted due to random wiring:
  5, 13, 2, 7, 12, 6, 10, 4, 8, 9, 3, 11
};

FLASH_STRING(status_n, "STATUS N");
FLASH_STRING(fast_white_select, "TWK 245 10 1 41 42 43 44 45 198 199");
// FIXME: other things to test:
//FLASH_STRING(off_select, "OFF 80 81 82 83");
FLASH_STRING(all_off, "OFF");

FLASH_STRING(dim_white_field, "CST 200 0 10");
FLASH_STRING(select_white_twinkle_1,
  "TWK 245 30 1 130 131 132 134 135 136 137 139 140 141 143 145 146 149");
FLASH_STRING(select_white_twinkle_2,
  "TWK 245 30 1 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79");
FLASH_STRING(select_white_twinkle_3,
  "TWK 245 30 1 110 111 112 113 114 118 120 121 122 123 124 125 128 129");

FLASH_STRING(medium_fast_white, "TWK 245 30 1");
FLASH_STRING(sparse_blue, "TWK 225 70 0");
FLASH_STRING(all_white, "CST 200 0 200");
FLASH_STRING(crazy_purple, "TWK 255 1 3");

FLASH_STRING(medium_fast_white_odd, "TWK 245 30 1 255");
FLASH_STRING(sparse_blue_odd, "TWK 225 70 0 255");
FLASH_STRING(all_white_odd, "CST 200 0 200 255");
FLASH_STRING(crazy_purple_odd, "TWK 255 1 3 255");

FLASH_STRING(medium_fast_white_even, "TWK 245 30 1 254");
FLASH_STRING(sparse_blue_even, "TWK 225 70 0 254");
FLASH_STRING(all_white_even, "CST 200 0 200 254");
FLASH_STRING(crazy_purple_even, "TWK 255 1 3 254");

FLASH_STRING(sleep_one_hour_odd, "SLEEP 60 60 255");
FLASH_STRING(sleep_one_hour_even, "SLEEP 60 60 254");

// TODO: status_n back in rotation?
TimedMessage twinkle_messages_odd[] = {
  { 4 * 60000, &medium_fast_white_odd },
  { 4 * 60000, &sparse_blue_odd },
  { 17000, &all_white_odd },
  { 17000, &crazy_purple_odd },
  { 4 * 60000, &sparse_blue_odd },
  { 5000,  &status_n },

  // Sleep others.
  { 5000,  &sleep_one_hour_even },

  // These aren't odd/even, but after the sleep message the others should be
  // sleeping.
  { 10000, &dim_white_field },
  { 20000, &select_white_twinkle_1 },
  { 8000, &dim_white_field },
  { 20000, &select_white_twinkle_2 },
  { 8000, &dim_white_field },
  { 20000, &select_white_twinkle_3 },
};

TimedMessage twinkle_messages_even[] = {
  { 4 * 60000, &medium_fast_white_even },
  { 4 * 60000, &sparse_blue_even },
  { 17000, &all_white_even },
  { 17000, &crazy_purple_even },
  { 4 * 60000, &sparse_blue_even },
  { 5000,  &status_n },

  // Sleep others.
  { 5000,  &sleep_one_hour_odd },

  // These aren't odd/even, but after the sleep message the others should be
  // sleeping.
  { 10000, &dim_white_field },
  { 20000, &select_white_twinkle_1 },
  { 8000, &dim_white_field },
  { 20000, &select_white_twinkle_2 },
  { 8000, &dim_white_field },
  { 20000, &select_white_twinkle_3 },
};

// Bootes
FLASH_STRING(constellation_0,
             //"CST 200 0 200 118 161 31 107 71 186 113 68");
             "TWK 255 1 3");
// Pegasus
FLASH_STRING(constellation_1,
             "CST 200 0 200 60 107 190 91 92 179 98 164 173");
// Little Dipper
FLASH_STRING(constellation_2,
             "CST 200 0 200 118 155 188 95 94 132 75");
// Draco
FLASH_STRING(constellation_3,
             "CST 200 0 200 121 96 162 106 107 60 104 173 161 140 143");
// Perseus: FIXME!
FLASH_STRING(constellation_4,
     //"CST 200 0 200 130 131 132 134 135 136 137 139 140 141 143 145 146 149");
     "CST 200 0 200");
// Not used.
FLASH_STRING(constellation_5, "CST 200 0 200 22 23 24");
FLASH_STRING(constellation_6, "CST 200 0 200 22 23 24");
FLASH_STRING(constellation_7, "CST 200 0 200 22 23 24");
// Andromeda
FLASH_STRING(constellation_8,
       "CST 200 0 200 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77");
// Canis Major  FIXME!
FLASH_STRING(constellation_9,
             //"CST 200 0 200 110 111 112 113 114 118 120 121 122 123 124 125");
             "TWK 245 30 1");
// Aquila
FLASH_STRING(constellation_10,
             "CST 200 0 200 39 153 36 173 92 64 75 164 3");
// Hercules.  FIXME incomoplete.  Button broken too.
FLASH_STRING(constellation_11, "CST 200 0 200 161 92 43 134");

#define NUM_CONSTELLATIONS 12
TimedMessage constellation_sequence_0[] = 
{
  { 0, &constellation_0 },
};
TimedMessage constellation_sequence_1[] = {
  { 0, &constellation_1 },
};
TimedMessage constellation_sequence_2[] = {
  { 0, &constellation_2 },
};
TimedMessage constellation_sequence_3[] = 
{
  { 0, &constellation_3 },
};
TimedMessage constellation_sequence_4[] = {
  { 0, &constellation_4 },
};
TimedMessage constellation_sequence_5[] = {
  { 0, &constellation_5 },
};
TimedMessage constellation_sequence_6[] = 
{
  { 0, &constellation_6 },
};
TimedMessage constellation_sequence_7[] = {
  { 0, &constellation_7 },
};
TimedMessage constellation_sequence_8[] = {
  { 0, &constellation_8 },
};
TimedMessage constellation_sequence_9[] = 
{
  { 0, &constellation_9 },
};
TimedMessage constellation_sequence_10[] = {
  { 0, &constellation_10 },
};
TimedMessage constellation_sequence_11[] = {
  { 0, &constellation_11 },
};
// If you change the number of sequences, update NUM_CONSTELLATIONS and
// InitConstellationSequenceArray().

FLASH_STRING(sleep_one_hour, "SLEEP 60 60");
FLASH_STRING(sleep_5_mins, "SLEEP 5 60");

TimedMessage bedtime_sequence[] = {
  { 4000, &all_off },
  // Sleep one hour, indefinitely.
  { 0, &sleep_one_hour },
};

TimedMessage standby_sequence[] = {
  { 4000, &all_off },
  { 0, &sleep_5_mins },
};

// How would star wars work here? I don't think the MessageTimer class will
// support it.  Probably move the star wars code from the Master into here.

DayCycle day_cycle_;

RTC_DS1307 RTC;

struct TransitionTime {
  // Start time in seconds since midnight.
  unsigned long start_time;
  byte day_cycle_state;
};

TransitionTime transitions_[NUM_DAY_CYCLE_STATES] = {
  { 19UL * 3600UL + 45UL * 60UL, ACTIVE },
  { 23UL * 3600UL + 55UL * 60UL, SLEEPING },
  { 18UL * 3600UL + 45UL * 60UL, STANDBY }
};

MessageTimer message_timer_;

#define LED_STRIP_PIN 47

#define STATUS_INTERVAL_MS 10000UL
unsigned long last_status_report_ = 0;

// Master-Mega communication happens over this port:
HardwareSerial* master_serial = &Serial1;

// We can afford a larger buffer on the mega.  Make it big enough to handle
// error logging and debug packet printing from the master.
#define BUFFLEN 1024
byte serialData[BUFFLEN];

void MaybeReadMasterSerial() {
  if (ReadMessageStringFromSerial(master_serial, serialData, BUFFLEN)) {
    String msg = "M ";
    msg += ((char*)serialData);
    Log(msg);
  }
}

byte next_status_id_ = MIN_SLAVE_ID;

// Replace the special command "STATUS N" with "STATUS <id>", where <id>
// is next_status_id_, which is then incremented.
String MaybeReplaceStatusId(String& message) {
  if (message != "STATUS N") {
    return message;
  }
  String id_string = String(next_status_id_, DEC);
  String msg_str = message.replace("N", id_string);
  ++next_status_id_;
  if (next_status_id_ > MAX_SLAVE_ID) {
    next_status_id_ = MIN_SLAVE_ID;
  }
  return msg_str;
}

// The master needs to be in MESSAGE_SERIAL mode for this.
inline void SendMessageToMaster() {
  // TODO(madadam): Avoid all these string copies.  Arduino's String class
  // doesn't make it easy.
  String message_string = message_timer_.GetCurrentMessage();
  message_string = MaybeReplaceStatusId(message_string);
  master_serial->println(message_string);
}

void InitDayCycleTransitions() {
  for (int i = 0; i < NUM_DAY_CYCLE_STATES; ++i) {
    if (!day_cycle_.SetStart(transitions_[i].day_cycle_state,
                             transitions_[i].start_time)) {
      Serial.print("Error in DayCycle transition spec ");
      Serial.println(i, DEC);
    }
  }
}

struct ConstellationSequence {
  byte length;
  TimedMessage* sequence;
};

ConstellationSequence constellation_sequences[NUM_CONSTELLATIONS];

// Awful. But multidimensional arrays can't have unspecified bound sizes
// except for the first dimension, so a normal initializer doesn't work.
void InitConstellationSequenceArray() {
  constellation_sequences[0].length =
      sizeof(constellation_sequence_0) / sizeof(TimedMessage);
  constellation_sequences[0].sequence = constellation_sequence_0;

  constellation_sequences[1].length =
      sizeof(constellation_sequence_1) / sizeof(TimedMessage);
  constellation_sequences[1].sequence = constellation_sequence_1;

  constellation_sequences[2].length =
      sizeof(constellation_sequence_2) / sizeof(TimedMessage);
  constellation_sequences[2].sequence = constellation_sequence_2;

  constellation_sequences[3].length =
      sizeof(constellation_sequence_3) / sizeof(TimedMessage);
  constellation_sequences[3].sequence = constellation_sequence_3;

  constellation_sequences[4].length =
      sizeof(constellation_sequence_4) / sizeof(TimedMessage);
  constellation_sequences[4].sequence = constellation_sequence_4;

  constellation_sequences[5].length =
      sizeof(constellation_sequence_5) / sizeof(TimedMessage);
  constellation_sequences[5].sequence = constellation_sequence_5;

  constellation_sequences[6].length =
      sizeof(constellation_sequence_6) / sizeof(TimedMessage);
  constellation_sequences[6].sequence = constellation_sequence_6;

  constellation_sequences[7].length =
      sizeof(constellation_sequence_7) / sizeof(TimedMessage);
  constellation_sequences[7].sequence = constellation_sequence_7;

  constellation_sequences[8].length =
      sizeof(constellation_sequence_8) / sizeof(TimedMessage);
  constellation_sequences[8].sequence = constellation_sequence_8;

  constellation_sequences[9].length =
      sizeof(constellation_sequence_9) / sizeof(TimedMessage);
  constellation_sequences[9].sequence = constellation_sequence_9;

  constellation_sequences[10].length =
      sizeof(constellation_sequence_10) / sizeof(TimedMessage);
  constellation_sequences[10].sequence = constellation_sequence_10;

  constellation_sequences[11].length =
      sizeof(constellation_sequence_11) / sizeof(TimedMessage);
  constellation_sequences[11].sequence = constellation_sequence_11;
};

// Declare a global DateTime even though we don't need it, to work around a
// compiler bug in Arduino 22.
DateTime date_time_now_;

void AppendFormattedDate(String* msg) {
  // Work around Arduino 22 bug instead of passing param:
  const DateTime& dt = date_time_now_;
  *msg += long(dt.year());
  *msg += "/";
  *msg += long(dt.month());
  *msg += "/";
  *msg += long(dt.day());
  *msg += " ";
  *msg += long(dt.hour());
  *msg += ":";
  *msg += long(dt.minute());
  *msg += ":";
  *msg += long(dt.second());
}

void MaybeReportStatus(unsigned long now) {
  // Work around Arduino 22 bug instead of passing param:
  const DateTime& dt_now = date_time_now_;
  if (IsTimerExpired(now, &last_status_report_, STATUS_INTERVAL_MS)) {
    String msg;
    msg += "G ";
    AppendFormattedDate(&msg);
    msg += " ";
    msg += String(now, DEC);
    msg += " ";
    msg += String(day_cycle_.state(), DEC);
    Log(msg);
  }
}

void LedTestPattern() {
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    int pin = button_led_pins[i];
    Serial.println(pin, DEC);
    digitalWrite(pin, HIGH);
    delay(600);
    digitalWrite(pin, LOW);
  }
}

bool sd_card_ok_;

// As per Arduino documentation about SD reader on a Mega.
#define SD_CHIP_SELECT_PIN 53

bool InitSdLog() {
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
    return false;
  }
  Serial.println("Initialized SD card.");
  return true;
}

File logfile_;

bool OpenLogFile() {
  if (!sd_card_ok_) return false;
  logfile_ = SD.open("logfile.txt", FILE_WRITE); // really append.
  return (logfile_ != NULL);
}

bool Log(const String& msg) {
  Serial.println(msg);
  if (OpenLogFile()) {
    logfile_.println(msg);
    logfile_.close();
  }
}

// FIXME scaffold
void SDReaderTest() {
  if (!sd_card_ok_) return;
  File testfile = SD.open("testfile.txt", FILE_READ);
  if (!testfile) {
    Serial.println("Couldnt open file.");
  }
  while (testfile.available()) {
    Serial.print(testfile.read(), BYTE);
  }
  testfile.close();
  delay(2000);
}

// FIXME scaffold
void LedStripTest() {
  digitalWrite(LED_STRIP_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_STRIP_PIN, LOW);
  delay(1000);
}

// FIXME scaffold
void FindButtonLedPin() {
  int low = 22; int hi = 32;
  for (int i = low; i < hi; ++i) {
    pinMode(i, OUTPUT);
  }
  for (int i = low; i < hi; ++i) {
    Serial.println(i, DEC);
    digitalWrite(i, HIGH);
    delay(1000);
    digitalWrite(i, LOW);
  }
}

// FIXME scaffold
void SignalTest() {
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    int val = digitalRead(button_signal_pins[i]);
    analogWrite(button_led_pins[i], 255 * val);
    if (val) {
      Serial.print(i, DEC);
      Serial.print(" ");
      Serial.print(button_signal_pins[i], DEC);
      Serial.print(" -> ");
      Serial.println(val, DEC);
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  // FIXME: Why is this corrupt?
  Serial.println(versionblurb);

  Serial1.begin(9600);

  Wire.begin();
  if (!RTC.begin()) {
    Serial.println("RTC init failed.");
  }

  //if (true) {
  if (!RTC.isrunning()) {
    DateTime date_compiled(__DATE__, __TIME__);
    RTC.adjust(date_compiled);
    Serial.print("Set RTC clock to ");
    Serial.println(date_compiled.unixtime(), DEC);
  }

  sd_card_ok_ = InitSdLog();

  InitDayCycleTransitions();

  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(button_signal_pins[i], INPUT);
    pinMode(button_led_pins[i], OUTPUT);
  }
  pinMode(LED_STRIP_PIN, OUTPUT);

  //LedTestPattern();

  InitConstellationSequenceArray();

  delay(1000);
  InitActiveCycle();
}

// A twinkler to fade out the button LEDs when a different button is pushed.
class ButtonFadeTwinkler : public Twinkler {
 public:
  ButtonFadeTwinkler() {
    // Start lit up.
    fade_direction_ = 0;
    duration_ = 0;
    // Use BLUE for the button LED.  Doesn't really matter which, we just need
    // to pick one since the button only has one LED (as opposed to the puck,
    // which is why the Twinkler supports two).
    use_led_[BLUE] = 1;
    use_led_[WHITE] = 0;
    SetSteadyState(MAX_FADE_VALUE);
  }

  virtual char* Name() const { return "ButtonTwinkler"; }

  // Turn on or off.
  void SetSteadyState(int value) {
    fade_direction_ = 0;
    duration_ = 0;
    fade_increment_ = 0;
    value_[BLUE] = float(value);
  }

#define FADE_OUT -1
#define FADE_IN 1
  void StartFade(float duration_sec, int direction) {
    fade_direction_ = direction;
    int intervals_per_sec = 1000 / FADE_INTERVAL_MS;
    duration_ = duration_sec * intervals_per_sec;
    brightness_ = MAX_FADE_VALUE;
    fade_increment_ = (float)brightness_ / duration_;
    if (direction == FADE_OUT) {
      value_[BLUE] = MAX_FADE_VALUE;
    } else {
      value_[BLUE] = MIN_FADE_VALUE;
    }
  }

  virtual void Update() {
    if (duration_ == 0) {
      return;
    }
    --duration_;
    IncrementValues(false, false);  // Don't reverse direction.
  }
};

#define BUTTON_LED_FADE_TIME_SEC 1.0

enum ButtonPanelState {
  BP_ON,
  BP_OFF,
  BP_ONE_SELECTED,
  NUM_BUTTON_PANEL_STATES
};

class Button {
 public:
  Button() {}

  void SetPins(byte signal_pin, byte led_pin) {
    signal_pin_ = signal_pin;
    led_pin_ = led_pin;
  }

  int Read() {
    return digitalRead(signal_pin_);
  }

  // Call this every FADE_INTERVAL_MS.
  void Update(unsigned long now) {
    if (twinkler_.shouldWrite(BLUE)) {
      int val = twinkler_.Value(BLUE);
      /*
      Serial.print("writing pin ");
      Serial.print(led_pin_, DEC);
      Serial.print(" val ");
      Serial.println(val, DEC);  // FIXME debug
      */

      analogWrite(led_pin_, val);
    }
    twinkler_.Update();
  }

  void TransitionToState(byte state, bool is_selected) {
    if (state == BP_ON) {
      if (current_state_ == BP_ONE_SELECTED) {
        twinkler_.StartFade(BUTTON_LED_FADE_TIME_SEC, FADE_IN);
      } else {
        // Or we could always fade in.
        twinkler_.SetSteadyState(MAX_FADE_VALUE);
      }
    } else if (state == BP_OFF) {
      twinkler_.SetSteadyState(MIN_FADE_VALUE);
    } else if (state == BP_ONE_SELECTED) {
      if (is_selected) {
        twinkler_.SetSteadyState(MAX_FADE_VALUE);
      } else {
        twinkler_.StartFade(BUTTON_LED_FADE_TIME_SEC, FADE_OUT);
      }
    }
    current_state_ = state;
  }

 private:
  byte signal_pin_;
  byte led_pin_;
  byte current_state_;
  ButtonFadeTwinkler twinkler_;
};

enum StarMode {
  MODE_TWINKLE,
  MODE_CONSTELLATION,
  MODE_OFF,
};

// Keep the array of buttons?
class ButtonController {
 public:
  ButtonController() {
    for (int i = 0; i < NUM_BUTTONS; ++i) {
      buttons_[i].SetPins(button_signal_pins[i], button_led_pins[i]);
    }
    Init();
  }

  void Init() {
    selected_button_ = -1;
    last_led_control_time_ = 0;
  }

  void MaybeRunLedControl(unsigned long now) {
    if (IsTimerExpired(now, &last_led_control_time_, FADE_INTERVAL_MS)) {
      for (int i = 0; i < NUM_BUTTONS; ++i) {
        buttons_[i].Update(now);
      }
    }
  }

  int selected_button() { return selected_button_; }

  // Clears the selected button and then polls the button signals.  Returns
  // true iff a button is currently signalled, and sets selected_button.
  bool PollButtons();

  // TODO(madadam): Configurable state transition diagram.
  void TransitionToState(byte new_state) {
    current_state_ = new_state;
    if (current_state_ != BP_ONE_SELECTED) {
      // Clear the selection.
      selected_button_ = -1;
    }
    for (int i = 0; i < NUM_BUTTONS; ++i) {
      bool is_selected = (i == selected_button_);
      buttons_[i].TransitionToState(current_state_, is_selected);
    }
  }

private:
  unsigned long last_led_control_time_;
  Button buttons_[NUM_BUTTONS];
  byte current_state_;
  int selected_button_;
};

bool ButtonController::PollButtons() {
  byte button_state[NUM_BUTTONS];
  selected_button_ = -1;
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    button_state[i] = buttons_[i].Read();
    if (button_state[i] == HIGH) {
      Serial.print("hi button "); Serial.println(i, DEC);  // FIXME debug
      // For now, we only handle single button presses at a time.
      selected_button_ = i;
    }
  }
  return selected_button_ >= 0;
}

#define CONSTELLATION_TIME_SEC 16

class ModeController {
 public:
  ModeController() {
    StartTwinkleMode();
  }

  void ModeControl(unsigned long now) {
    if (star_mode_ == MODE_CONSTELLATION) {
      MaybeChangeMode(now);
    }

    if (star_mode_ == MODE_TWINKLE) {
      if (button_controller_.PollButtons()) {
        StartConstellationMode(now);
      }
    }

    button_controller_.MaybeRunLedControl(now);
  }

  void StartConstellationMode(unsigned long now) {
    // Start the selected constellation mode message timer.
    int selected_button = button_controller_.selected_button();
    if (selected_button >= NUM_CONSTELLATIONS) {
      Serial.println("Constellation # out of range.");
      return;
    }

    Serial.println("G MODE_CONSTELLATION");
    // Start mode timer and transition.
    mode_timer_start_ = now;
    star_mode_ = MODE_CONSTELLATION;
    button_controller_.TransitionToState(BP_ONE_SELECTED);

    byte num_msgs = constellation_sequences[selected_button].length;
    String msg("G const ");
    msg += long(selected_button);
    Log(msg);
    message_timer_.StartWithMessages(
        constellation_sequences[selected_button].sequence, num_msgs);
    SendMessageToMaster();
  }

  void StartTwinkleMode() {
    Serial.println("G MODE_TWINKLE");
    star_mode_ = MODE_TWINKLE;
    // FIXME: This should clear the button signal poll, so that we don't
    // read a button press that happened during constellation mode.
    button_controller_.TransitionToState(BP_ON);
    mode_timer_start_ = 0;

    // Set the master back to regular twinkle pattern.
    const DateTime& now = date_time_now_;
    if ((now.day() % 2) == 0) {
      byte num_msgs =
          (byte)(sizeof(twinkle_messages_even)/sizeof(TimedMessage));
      message_timer_.StartWithMessages(twinkle_messages_even, num_msgs);
    } else {
      byte num_msgs =
          (byte)(sizeof(twinkle_messages_odd)/sizeof(TimedMessage));
      message_timer_.StartWithMessages(twinkle_messages_odd, num_msgs);
    }
    SendMessageToMaster();
  }

  void StartInactiveMode() {
    Serial.println("G MODE_OFF");
    star_mode_ = MODE_OFF;
    button_controller_.TransitionToState(BP_OFF);
    mode_timer_start_ = 0;
  }

  bool MaybeChangeMode(unsigned long now) {
    if (mode_timer_start_ > 0 &&
        IsTimerExpired(now,
                       &mode_timer_start_,
                       1000 * CONSTELLATION_TIME_SEC)) {
      if (star_mode_ == MODE_CONSTELLATION) {
        StartTwinkleMode();
      }
      // Stop timer.
      mode_timer_start_ = 0;
      return true;
    }
    return false;
  }

 private:
  byte star_mode_;
  unsigned long mode_timer_start_;
  ButtonController button_controller_;
};

ModeController mode_controller_;

void InitActiveCycle() {
  mode_controller_.StartTwinkleMode();
  digitalWrite(LED_STRIP_PIN, HIGH);
}

void InitSleepCycle() {
  byte num_msgs = (byte)(sizeof(bedtime_sequence)/sizeof(TimedMessage));
  message_timer_.StartWithMessages(bedtime_sequence, num_msgs);
  SendMessageToMaster();

  // Turn off the button LEDs etc.
  mode_controller_.StartInactiveMode();
  digitalWrite(LED_STRIP_PIN, LOW);
}

void InitStandbyCycle() {
  byte num_msgs = (byte)(sizeof(standby_sequence)/sizeof(TimedMessage));
  message_timer_.StartWithMessages(standby_sequence, num_msgs);
  SendMessageToMaster();

  // Turn off the button LEDs etc.
  mode_controller_.StartInactiveMode();
  digitalWrite(LED_STRIP_PIN, LOW);
}

inline unsigned long RtcSecondsSinceMidnight() {
  // Work around Arduino 22 bug, instead of passing param:
  const DateTime& now = date_time_now_;
  unsigned long secs = (unsigned long)(now.hour()) * 3600UL +
                       (unsigned long)(now.minute()) * 60UL +
                       now.second();
  return secs;
}

void loop() {
  // FIXME scaffold
  //SignalTest();
  //FindButtonLedPin();
  //LedTestPattern();
  //LedStripTest();
  //SDReaderTest();

  date_time_now_ = RTC.now();
  unsigned long day_time = RtcSecondsSinceMidnight();

  if (day_cycle_.CheckForTransition(day_time)) {
    if (day_cycle_.state() == ACTIVE) {
      Serial.println("G Starting ACTIVE");
      InitActiveCycle();
    } else if (day_cycle_.state() == SLEEPING) {
      Serial.println("G Starting SLEEP");
      InitSleepCycle();
    } else if (day_cycle_.state() == STANDBY) {
      Serial.println("G Starting STANDBY");
      InitStandbyCycle();
    }
  }

  unsigned long now = millis();
  mode_controller_.ModeControl(now);
  if (message_timer_.MaybeChangeMessage(now)) {
    SendMessageToMaster();
  }

  MaybeReadMasterSerial();

  MaybeReportStatus(now);

  delay(10); // FIXME
} 
