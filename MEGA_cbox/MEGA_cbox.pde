/******* Reflecting the Stars *********
Control Box.  Runs on the Arduino Mega.
Author: Adam Berenzweig
----------
**************************************/

char* versionblurb = "v.1.0 - Control Box"; 

#include <DayCycle.h>
#include "HardwareSerial.h"
#include <DateTime.h>
#include <MessageTimer.h>
#include <RtsMessage.h>
#include <RtsMessageParser.h>
#include <PrintUtil.h>
#include <RTClib.h>
#include <RtsUtil.h>
#include <SD.h>
#include <Twinkler.h>
#include <Wire.h>

#define NUM_BUTTONS 3

#define STATUS_INTERVAL_MS 10000UL
unsigned long last_status_report_ = 0;

int button_signal_pins[NUM_BUTTONS] = {
  22, 23, 24
};

int button_led_pins[NUM_BUTTONS] = {
  8, 9, 10,  // PWM pins
};

TimedMessage twinkle_messages[] = {
  //{ 10000, "TWK 215 60 0" },  // Sparse blue.
  //{ 10000, "TWK 215 60 1" },  // Sparse white.
  //{ 30000, "TWK 245 10 0" },  // Fast blue.
  { 30000, "TWK 245 10 0 255" },  // Fast blue odd.
  { 30000, "TWK 245 10 0 254" },  // Fast blue even.
  //{ 10000, "CST 200 0 200 50 51 52" },  // A constellation.
  //{ 10000, "CST 200 200 100 2" },  // A constellation.
  { 5000,  "STATUS 24" },
};

#define NUM_CONSTELLATIONS 3
TimedMessage constellation_sequence_0[] = 
{
  { 0, "CST 200 0 200 24" },
};
TimedMessage constellation_sequence_1[] = {
  { 0, "CST 200 0 200 22" },
};
TimedMessage constellation_sequence_2[] = {
  { 0, "CST 200 0 200 22 23 24" },
};
// If you change the number of sequences, update NUM_CONSTELLATIONS.

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
};

TimedMessage bedtime_sequence[] = {
  { 4000, "OFF" },
  { 0, "SLEEP 60 60" },  // Sleep one hour, indefinitely.
};

TimedMessage standby_sequence[] = {
  { 4000, "OFF" },
  { 0, "SLEEP 5 60" },  // Sleep 5 minutes.
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
  { 19UL * 3600UL + 55UL * 60UL, ACTIVE },
  { 23UL * 3600UL + 55UL * 60UL, SLEEPING },
  { 18UL * 3600UL + 50UL * 60UL, STANDBY }
};

MessageTimer message_timer_;

#define LED_STRIP_PIN 35

// As per Arduino documentation about SD reader on a Mega.
#define SD_CHIP_SELECT_PIN 53

// Master-Mega communication happens over this port:
HardwareSerial* master_serial = &Serial1;

// We can afford a larger buffer on the mega.  Make it big enough to handle
// error logging and debug packet printing from the master.
#define BUFFLEN 1024
byte serialData[BUFFLEN];

void MaybeReadMasterSerial() {
  if (ReadMessageStringFromSerial(master_serial, serialData, BUFFLEN)) {
    Serial.print("M ");
    Serial.println((char*)serialData);
  }
}

// The master needs to be in MESSAGE_SERIAL mode for this.
inline void SendMessageToMaster() {
  master_serial->println(message_timer_.current_message_string());
  // Serial.print("sent: ");  // FIXME debug scaffold
  // Serial.println(message_timer_.current_message_string());
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

// Declare a global DateTime even though we don't need it, to work around a
// compiler bug in Arduino 22.
DateTime date_time_now_;

void PrintDate(HardwareSerial* serial) {
  // Work around Arduino 22 bug instead of passing param:
  const DateTime& dt = date_time_now_;
  serial->print(dt.year(), DEC);
  serial->print("/");
  serial->print(dt.month(), DEC);
  serial->print("/");
  digitalWrite(LED_STRIP_PIN, LOW);
  digitalWrite(LED_STRIP_PIN, LOW);
  digitalWrite(LED_STRIP_PIN, LOW);
  digitalWrite(LED_STRIP_PIN, LOW);
  digitalWrite(LED_STRIP_PIN, LOW);
  serial->print(dt.day(), DEC);
  serial->print(" ");
  serial->print(dt.hour(), DEC);
  serial->print(":");
  serial->print(dt.minute(), DEC);
  serial->print(":");
  serial->print(dt.second(), DEC);
}

void MaybeReportStatus(unsigned long now) {
  // Work around Arduino 22 bug instead of passing param:
  const DateTime& dt_now = date_time_now_;
  if (IsTimerExpired(now, &last_status_report_, STATUS_INTERVAL_MS)) {
    //memrep();
    Serial.print("G ");
    PrintDate(&Serial);
    Serial.print(" ");
    Serial.print(now, DEC);
    Serial.print(" ");
    Serial.print(day_cycle_.state(), DEC);

    Serial.println();
  }
}

void LedTestPattern() {
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    int pin = button_led_pins[i];
    digitalWrite(pin, HIGH);
    delay(600);
    digitalWrite(pin, LOW);
  }
}

bool sd_card_ok_;
/*
// If using the SdFat library:
Sd2Card sd_card_;
SdVolume sd_volume_;
SdFile file_root_;

bool InitSdCard() {
  if (!sd_card_.init()) {
    Serial.println("Couldnt init SD card.");
    return false;
  }
  if (!sd_volume_.init(sd_card_)) {
    Serial.println("couldnt init SD volume");
    return false;
  }
  if (!file_root_.openRoot(sd_volume_)) {
    Serial.println("couldnt open SD root");
    return false;
  }
  Serial.println("Initialized SD card.");
  return true;
}

void SDReaderTest() {
  SdFile logfile;
  if (!logfile.open(file_root_, "testfile.txt", O_READ)) {
    Serial.println("Couldnt open file.");
  } else {
    int c;
    while ((c = logfile.read()) > 0) {
      Serial.print(c);
    }
    logfile.close();
  }
  delay(2000);
}
*/

bool InitSdCard() {
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println("Card failed, or not present");
    return false;
  }
  Serial.println("Initialized SD card.");
  return true;
}

// FIXME scaffold
void SDReaderTest() {
  if (!sd_card_ok_) return;
  File logfile = SD.open("testfile.txt", FILE_READ);
  if (!logfile) {
    Serial.println("Couldnt open file.");
  }
  while (logfile.available()) {
    Serial.print(logfile.read());
  }
  logfile.close();
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

  sd_card_ok_ = InitSdCard();

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

#define CONSTELLATION_TIME_SEC 20

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
    Serial.println("G MODE_CONSTELLATION");  // FIXME
    // Start mode timer and transition.
    mode_timer_start_ = now;
    star_mode_ = MODE_CONSTELLATION;
    button_controller_.TransitionToState(BP_ONE_SELECTED);

    // Start the selected constellation mode message timer.
    int selected_button = button_controller_.selected_button();
    byte num_msgs = constellation_sequences[selected_button].length;
    Serial.print("Transition to constellation ");
    Serial.print(selected_button, DEC);
    Serial.print(" with size ");
    Serial.println(num_msgs, DEC);
    message_timer_.StartWithMessages(
        constellation_sequences[selected_button].sequence, num_msgs);
    SendMessageToMaster();
  }

  void StartTwinkleMode() {
    Serial.println("G MODE_TWINKLE");  // FIXME
    star_mode_ = MODE_TWINKLE;
    // FIXME: This should clear the button signal poll, so that we don't
    // read a button press that happened during constellation mode.
    button_controller_.TransitionToState(BP_ON);
    mode_timer_start_ = 0;

    // Set the master back to regular twinkle pattern.
    byte num_msgs = (byte)(sizeof(twinkle_messages)/sizeof(TimedMessage));
    message_timer_.StartWithMessages(twinkle_messages, num_msgs);
    SendMessageToMaster();
  }

  void StartInactiveMode() {
    Serial.println("G MODE_OFF");  // FIXME
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
  SDReaderTest();

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
