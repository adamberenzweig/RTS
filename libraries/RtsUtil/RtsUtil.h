/******* Reflecting the Stars *********
Util
Version: 0.1
Authors: Adam Berenzweig
Date: 2010-12-26
----------
Utility functions.
**************************************/

#ifndef RTS_UTIL_H
#define RTS_UTIL_H 1

#include "WProgram.h"

class SmoothedThreshold {
 public:
  SmoothedThreshold() {}

  void Init(float window_length_sec,
            float low_threshold,
            float hi_threshold) {
    window_length_sec_ = window_length_sec;
    smoothed_value_ = 0;
    last_update_ms_ = 0;
    size_sec_ = 0;

    low_threshold_ = low_threshold;
    hi_threshold_ = hi_threshold;
    state_ = STATE_HIGH;
  }

  enum STATE {
    STATE_LOW,
    STATE_HIGH,
  };

  byte state() { return state_; }
  float smoothed_value() { return smoothed_value_; }

  // Returns true iff this update caused a state transition.
  bool Update(float value, unsigned long now_ms) {
    if (0 == last_update_ms_) {
      smoothed_value_ = value;
    } else {
      float elapsed_time =
          min(float(now_ms - last_update_ms_)/1000.0, window_length_sec_);
      if (size_sec_ < window_length_sec_) {
        size_sec_ += elapsed_time;
        size_sec_ = min(size_sec_, window_length_sec_);
      }
      // To do this correctly we'd need a buffer of the whole window, but to
      // save memory we just use a single value, so the running average is an
      // estimate.
      smoothed_value_ =
          (value * elapsed_time + smoothed_value_ * (size_sec_ - elapsed_time))
          / size_sec_;
    }

    last_update_ms_ = now_ms;

    if (smoothed_value_ < low_threshold_ && state_ == STATE_HIGH) {
      state_ = STATE_LOW;
      return true;
    } else if (smoothed_value_ >= hi_threshold_ && state_ == STATE_LOW) {
      state_ = STATE_HIGH;
      return true;
    }
    return false;
  }

 private:
  float smoothed_value_;
  float window_length_sec_;
  // Filled window size.
  float size_sec_;
  unsigned long last_update_ms_;

  float low_threshold_;
  float hi_threshold_;
  byte state_;
};

#endif  // RTS_UTIL_H
