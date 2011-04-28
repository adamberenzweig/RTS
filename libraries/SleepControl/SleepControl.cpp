/******* Reflecting the Stars *********
Radio Test - SLAVE CODE
Version: 0.1
Authors: Adam Berenzweig, Icing Chang
Date: 2010-08-15
----------
Library for the radio protocol between master and slave.
**************************************/

// #include <PrintUtil.h>
#include "WProgram.h"
#include "SleepControl.h"

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// NOTE: Keep these two constants in sync.
#define WATCHDOG_INTERVAL_MS 2000
byte kWdpSetting = WDTO_2S;
    
EMPTY_INTERRUPT(WDT_vect);
/*
ISR(WDT_vect) {
  Serial.println("Watchdog interrpt");
}
*/


void InitSleepControl() {
  setWatchdogInterrupt(1, kWdpSetting);
}

void setWatchdogInterrupt(byte enable, byte wdp_setting) {
  byte val;
  if (enable) {
    val = (1<<WDIE) | wdp_setting;
  } else {
    WDTCSR = 0;
  }
  
  MCUSR &= ~(1<<WDRF); // Clear the reset flag after an interrupt.
  cli();
  // NOTE! These two instructions must be executed within 4 cycles.
  // Any other instructions on these lines will screw things up, including
  // reading wdp_setting!
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = val;
  sei();
}

unsigned long sleepWithTimeout(unsigned long ms_to_sleep) {
  if (ms_to_sleep < WATCHDOG_INTERVAL_MS) {
    // Don't sleep for less than the watchdog interval.
    return 0;
  }
  
  unsigned long t0 = millis();

  unsigned long ticks_to_sleep = ms_to_sleep / WATCHDOG_INTERVAL_MS;
  for (unsigned long ticks = ticks_to_sleep; ticks > 0; --ticks) {
    sleepNow(SLEEP_MODE_PWR_DOWN);
    //Serial.print('z');
#ifdef PRODUCTION
    DFlashLeds(5);  // Flash blue while sleeping, so we know puck is alive.
#endif
  }
  // adjust the milli ticks, since we will have missed several
  extern volatile unsigned long timer0_millis;
  // We didn't sleep exactly ms_to_sleep, we slept an even number of ticks.
  timer0_millis += ticks_to_sleep * WATCHDOG_INTERVAL_MS;
  
  // DPrintUL("slept (ms)", millis() - t0);
  // DPrintln();
  
  return millis() - t0;
}

void sleepNow(byte mode) {
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
  * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
  * there is a list of sleep modes which explains which clocks and 
  * wake up sources are available in which sleep modus.
  *
  * In the avr/sleep.h file, the call names of these sleep modus are to be
  * found:
  *
  * The 5 different modes are:
  *     SLEEP_MODE_IDLE         -the least power savings 
  *     SLEEP_MODE_ADC
  *     SLEEP_MODE_PWR_SAVE
  *     SLEEP_MODE_STANDBY
  *     SLEEP_MODE_PWR_DOWN     -the most power savings
  *
  *  the power reduction management <avr/power.h>  is described in 
  *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  */  

  set_sleep_mode(mode);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin 

  // TODO(madadam): If we go into a deeper sleep than IDLE, I believe that
  // these are irrelevant, because the peripherals are already off.
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();

  sleep_mode();            // here the device is actually put to sleep!!

                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...

  power_all_enable();
}
