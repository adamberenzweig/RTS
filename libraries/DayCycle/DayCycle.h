/******* Reflecting The Stars ***********
DayCycle
Author: Adam Berenzweig
Date: 2011-07-30
*****************************************/

#include <RtsUtil.h>

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

#define ACTIVE_DURATION_MS 14400000UL    // 4 hours
#define SLEEPING_DURATION_MS 64800000UL // 18 hours
// If this is > 0, then only standby for the specified time.  Otherwise, standby
// indefinitely until the SOLAR_STATE transitions back to night.
#define STANDBY_DURATION_MS 7200000UL // 2 hours

// If nonzero, check the solar voltage once per interval, possibly transitioning
// to daytime sleep mode.
#define SOLAR_SLEEP_CHECK_INTERVAL_MS 5000

class DayCycle {
 public:
  DayCycle(byte solar_pin)
      : solar_pin_(solar_pin) {
    day_cycle_state_ = ACTIVE;
    last_solar_check_ = 0;
    last_cycle_transition_ = 0;
  }
  byte state() const { return day_cycle_state_; }

  // Return true iff we transitioned to a new state.
  // TODO(madadam): In production, this code or its equivalent will run in the
  // Mega.  Rip it out of here, or at least refactor to a library.
  bool CheckForTransition(unsigned long now) {
    if ((STANDBY_DURATION_MS > 0 &&
         day_cycle_state_ == STANDBY &&
         IsTimerExpired(now, &last_cycle_transition_, STANDBY_DURATION_MS)) ||
        (SolarTransition(now, &solar_state_) && solar_state_ == NIGHT)) {
      // Sun just set.  Wake up, little vampire.
      day_cycle_state_ = ACTIVE;
      return true;
    }
    if (day_cycle_state_ == ACTIVE &&
        IsTimerExpired(now, &last_cycle_transition_, ACTIVE_DURATION_MS)) {
      day_cycle_state_ = SLEEPING;
      return true;
    }
    if (day_cycle_state_ == SLEEPING &&
        IsTimerExpired(now, &last_cycle_transition_, SLEEPING_DURATION_MS)) {
      day_cycle_state_ = STANDBY;
      return true;
    }
    return false;
  }

  // TODO(madadam): Use SmoothedThreshold here.
  bool SolarTransition(unsigned long now, byte* solar_state) {
    if (SOLAR_SLEEP_CHECK_INTERVAL_MS > 0 &&
        IsTimerExpired(now,
                       &last_solar_check_,
                       SOLAR_SLEEP_CHECK_INTERVAL_MS)) {
      int solar_reading = analogRead(solar_pin_);
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

 private:
  byte solar_pin_;
  byte day_cycle_state_;
  // Last time we checked the solar voltage for solar-sleep.
  unsigned long last_solar_check_;
  unsigned long last_cycle_transition_;
};
