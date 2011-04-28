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

  void InitRelativeToInitialValue(int initial_value,
                                  float threshold_fraction,
                                  float histeresis_fraction,
                                  int window_length) {
    int threshold = initial_value * threshold_fraction;
    int histeresis = initial_value * histeresis_fraction;
    Init(initial_value, threshold, histeresis, window_length);
  }

  // The initial state is low iff initial_value < threshold.
  // The initial value counts as one point in the buffer.
  void Init(int initial_value, int threshold, int histeresis,
            int window_length) {
    window_length_ = window_length;
    running_average_ = initial_value;
    num_points_ = 1;
    low_threshold_ = threshold - histeresis;
    hi_threshold_ = threshold + histeresis;
    if (initial_value < threshold) {
      state_ = STATE_LOW;
    } else {
      state_ = STATE_HIGH;
    }
  }

  enum STATE {
    STATE_LOW,
    STATE_HIGH,
  };

  byte state() { return state_; }

  float smoothed_value() { return running_average_; }

  // FIXME
  int low_threshold() { return low_threshold_; }

  // Returns true iff this update caused a state transition.
  bool Update(int value) {
    if (num_points_ < window_length_) { num_points_++; }
    // To do this correctly we'd need a buffer of the whole window, but to save
    // memory we just use a single value, so the running average is an estimate.
    // As it is, this takes 56 bytes per object w/ class overhead.
    running_average_ = (value + running_average_ * (num_points_ - 1))
			/ num_points_;

    if (running_average_ < low_threshold_ && state_ == STATE_HIGH) {
      state_ = STATE_LOW;
      return true;
    } else if (running_average_ > hi_threshold_ && state_ == STATE_LOW) {
      state_ = STATE_HIGH;
      return true;
    }
    return false;
  }

 private:
  int low_threshold_;
  int hi_threshold_;
  int window_length_;

  float running_average_;
  int num_points_;
  byte state_;
};

#endif  // RTS_UTIL_H
