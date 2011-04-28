/******* Reflecting the Stars *********
Twinkler
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-08-16
----------
**************************************/

#include <PrintUtil.h>
#include "WProgram.h"
#include "Twinkler.h"


Twinkler::Twinkler() {
  Reset();
}

void Twinkler::Reset() {
  for (int i = 0; i < NUM_LEDS; ++i) {
    value_[i] = 0;
    use_led_[i] = 0;
  }
}

void RandomTwinkler::Init(const RtsMessage& message) {
  speed_ = message.getParam(TWINKLE_SPEED);
  off_time_ = message.getParam(TWINKLE_OFF_TIME);
  color_ = message.getParam(TWINKLE_COLOR);
  next_state_ = random(NUM_RANDOM_TWINKLE_STATES);

  InitRandomTwinkle(speed_, off_time_, color_);
}

// Constructor without a message, just pick some params.
void RandomTwinkler::Init() {
  speed_ = 255;
  off_time_ = 30;
  color_ = random(3);
  next_state_ = random(NUM_RANDOM_TWINKLE_STATES);

  InitRandomTwinkle(speed_, off_time_, color_);
}

float ScaleLinearly(float min, float max, byte value) {
  return ((max - min)/255.0) * value + min;
}

void ConstellationTwinkler::Init(const RtsMessage& message) {
  Reset();
  byte fade_in = message.getParam(STAR_FADE_IN);
  byte blue_brightness = message.getParam(STAR_BLUE_BRIGHTNESS);
  byte white_brightness = message.getParam(STAR_WHITE_BRIGHTNESS);

  fade_direction_ = 1;
  if (blue_brightness > 0) {
    use_led_[BLUE] = 1;
  }
  if (white_brightness > 0) {
    use_led_[WHITE] = 1;
  }

  if (fade_in == 0) {
    duration_ = 0;
    value_[BLUE] = blue_brightness * 1.0;
    value_[WHITE] = white_brightness * 1.0;
    blue_increment_ = 0;
    white_increment_ = 0;
  } else {
    // Fade-in range is 0 to 10 seconds.
    float duration_sec = ScaleLinearly(0.0, 10.0, fade_in); 
    // That's in seconds. Scale to 30ms intervals.
    int intervals_per_sec = 1000 / FADE_INTERVAL_MS;
    duration_ = duration_sec * intervals_per_sec;
    blue_increment_ = (blue_brightness * 1.0) / duration_;
    white_increment_ = (white_brightness * 1.0) / duration_;
  }
}

void ConstellationTwinkler::InitDark() {
  Reset();
  fade_direction_ = 0;
  duration_ = 0;
  blue_increment_ = 0;
  white_increment_ = 0;
}

float randFloat(float min, float max) {
  long r = random(1000);
  return min + (float)r / 1000.0 * (max - min);
}

void RandomTwinkler::InitRandomTwinkle(
    byte speed, byte off_time, byte color_mode) {
  Reset();

  // Duration counts in 30ms increments.
  // Pick a random duration as follows:  Compute the center (mean) by scaling
  // the speed paramter into the range [0.25, 18] seconds.  Then choose a random
  // duration uniformly from [1/2 * center, 3/2 * center].
  //
  // For a table of values of the center as a function of speed, use this
  // python:
  // for i in range(0,256, 10):
  //   print i, i*(18-0.25)/255.0 + 0.25

  float center;
  if (next_state_ == STATE_LEDS_ON) {
    center = ScaleLinearly(0.25, 18.0, 255 - speed);
  } else if (next_state_ == STATE_LEDS_OFF) {
    center = ScaleLinearly(0.25, 18.0, off_time);
  }
  float variance = center * 0.50;

  float duration_sec = center + randFloat(-1.0 * variance, variance);
  if (duration_sec < 0) {
    duration_sec = 0.03;  // One interval
  }
  // That's in seconds. Scale to 30ms intervals.
  int intervals_per_sec = 1000 / FADE_INTERVAL_MS;
  duration_ = duration_sec * intervals_per_sec;

  // Brightness is always random in [50,255].
  brightness_ = random(50, 256);
  // We are fading from 0 to brightness, so figure out the fade increment:
  fade_increment_ = (float)brightness_ / duration_;

  // Now double the duration because we're fading in and then back out.
  // TODO: Should half duration_sec above so the total time is controlled
  // correctly by the speed param.
  // TODO: make fade in/out optional, with a bit in the 3rd param.
  duration_ *= 2;

  if (next_state_ == STATE_LEDS_ON) {
    // Fade in first.
    fade_direction_ = 1;
  } else {
    fade_direction_ = 0;
  }
  // Flip the state.
  next_state_ = (next_state_ == STATE_LEDS_OFF) ? STATE_LEDS_ON
      : STATE_LEDS_OFF;

  switch(color_mode) {
  case(0):
    use_led_[BLUE] = 1;
    break;
  case(1):
    use_led_[WHITE] = 1;
    break;
  case(2):
    // Random color.
    use_led_[random(NUM_LEDS)] = 1;
    break;
  case(3):
    use_led_[BLUE] = 1;
    use_led_[WHITE] = 1;
    break;
  default:
    use_led_[WHITE] = 1;
    break;
  }
}

void Twinkler::DebugPrint() const {
  DPrint(Name());
  DPrintInt(" vB", value_[0]);
  DPrintInt(" vW", value_[1]);
  DPrintByte(" dir", fade_direction_);
  DPrintInt(" duration", duration_);
}

void ConstellationTwinkler::DebugPrint() const {
  Twinkler::DebugPrint();
  if (use_led_[BLUE]) {
    DPrintFloat(" blue", blue_increment_);
  }
  if (use_led_[WHITE]) {
    DPrintFloat(" white", white_increment_);
  }
  DPrintln();
}

void RandomTwinkler::DebugPrint() const {
  Twinkler::DebugPrint();
  if (use_led_[BLUE]) {
    DPrint(" blue");
  }
  if (use_led_[WHITE]) {
    DPrint(" white");
  }
  DPrintFloat(" inc", fade_increment_);
  DPrintln();
}

void ConstellationTwinkler::Update() {
  if (duration_ == 0) {
    // We're at the steady state, nothing to change.
    return;
  }
  if (use_led_[BLUE]) {
    value_[BLUE] += blue_increment_ * fade_direction_;
  }
  if (use_led_[WHITE]) {
    value_[WHITE] += white_increment_ * fade_direction_;
  }
  --duration_;
}

void RandomTwinkler::Update() {
  if (duration_ == 0) {
    // Start again with new random values.
    InitRandomTwinkle(speed_, off_time_, color_);
    return;
  }
  --duration_;

  IncrementValues(true, false); // reverse at max, not at min.
}

void Twinkler::IncrementValues(bool should_reverse_at_max,
                               bool should_reverse_at_min) {
  bool hit_min = false;
  bool hit_max = false;
  for (int i = 0; i < NUM_LEDS; ++i) {
    if (use_led_[i]) {
      value_[i] += fade_increment_ * fade_direction_;
      if (value_[i] < MIN_FADE_VALUE) {
        value_[i] = MIN_FADE_VALUE;
        hit_min = true;
      }
      if (value_[i] >= brightness_) {
        value_[i] = brightness_;
        hit_max = true;
      }
    }
  }
  if (hit_max) {
    if (should_reverse_at_max) {
      fade_direction_ *= -1;
    } else {
      // Stop the fade-in, go to steady-state, even if duration isn't quite
      // done yet.
      fade_direction_ = 0;
      duration_ = 0;
    }
  }
  if (hit_min) {
    if (should_reverse_at_min) {
      fade_direction_ *= -1;
    } else {
      // Stop the fade-out.
      fade_direction_ = 0;
    }
  }
}
