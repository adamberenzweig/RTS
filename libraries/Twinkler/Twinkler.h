/******* Reflecting the Stars *********
Twinkler
Version: 0.1
Authors: Adam Berenzweig
Date: August 16, 2010
----------------------
Controls the twinkling of an individual puck.
**************************************/

#ifndef TWINKLER_H 
#define TWINKLER_H 1

#include <RtsMessage.h>
#include "WProgram.h"

#define FADE_INTERVAL_MS 30
#define STANDARD_FADE_TIME_MS 1000
#define MAX_FADE_VALUE 255
#define MIN_FADE_VALUE 0

enum LEDS {
  BLUE,
  WHITE,
  NUM_LEDS
};

class Twinkler {
 public:
  Twinkler();

  virtual void Reset();
  
  // Every FADE_INTERVAL_MS, we are asked for the Value() to set the LED
  // brightness, and then Update() is called.
  // Base class does nothing.
  virtual void Update() {}
  
  // Return the desired value for the selected LED.
  virtual int Value(byte led) const {
    return int(value_[led]);
  }

  // Return true if the state of the led should be written to the pin.
  virtual byte shouldWrite(byte led) const {
    // TODO: only write if there is a change, maybe that saves power.
    return true;
  }
  
  virtual char* Name() const { return ""; }
  virtual void DebugPrint() const;
   
 protected:
  void IncrementValues(bool should_reverse_at_max, bool should_reverse_at_min);

  // Current value of the LED pin.
  // Kept as float for accuracy, but converted to int when Value() is called.
  float value_[NUM_LEDS];

  // Brightness parameter (not necessarily current brightness).
  byte brightness_;
  
  // which LEDs to light. if led_on_[BLUE] is true, light blue, etc.
  byte use_led_[NUM_LEDS];

  // Variables for fading in/out:

  // Every FADE_INTERVAL_MS, we will increase (or decrease) the fade_value
  // by this much:
  float fade_increment_;
  
  // How many 30ms intervals to last before re-Init.
  int duration_;
  
  // Are we fading in (1) or fading out (-1), or not fading (0)
  int fade_direction_;
  
};

class RandomTwinkler : public Twinkler {
 public:
  RandomTwinkler() {}
  // Init using params from the message:
  void Init(const RtsMessage& message);
  // A no-arg init that uses default params:
  void Init();

  virtual char* Name() const { return "RandomTwinkler"; }
  virtual void Update();
  virtual void DebugPrint() const;

 protected:
  // Pick random values based on the speed and brightness params.
  void InitRandomTwinkle(byte speed, byte brightness, byte color);

  enum RANDOM_TWINKLE_STATES {
    STATE_LEDS_ON,
    STATE_LEDS_OFF,
    NUM_RANDOM_TWINKLE_STATES
  };
  byte next_state_;

  // In addition to brightness_ in the base class, some other params.
  byte speed_;
  byte off_time_;
  byte color_;
};

class ConstellationTwinkler : public Twinkler {
 public:
  ConstellationTwinkler() {}
  void Init(const RtsMessage& message);
  // Used to implement the lights-off state:
  void InitDark();
  virtual char* Name() const { return "ConstlTwinkler"; }
  virtual void Update();
  virtual void DebugPrint() const;

 private:
  // Constellation mode needs separate blue and white increments.
  float blue_increment_;
  float white_increment_;
};

// A factory method to construct a new Twinkler of the right subclass based
// on the LED_STATE, and params from the message.
//Twinkler* TwinklerFactory(byte led_state, const RtsMessage& message);

#endif  // TWINKLER_H
