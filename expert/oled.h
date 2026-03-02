
#ifndef _OLED_H
#define _OLED_H

#include <Arduino.h>
#include <PololuOLED.h>


/* 
 *  This class is a wrapper written by Paul O'Dowd
 *  to expose the OLED functionality and provide a
 *  count down timer of 4 minutes.
 *  
 *  To use this, please follow the instructions in
 *  Supplementary labsheet 4.
 */
class OLED_c: public PololuSH1106 {

  public:
    OLED_c( uint8_t clk, uint8_t mos, uint8_t res, uint8_t dc, uint8_t cs ): PololuSH1106(clk, mos, res, dc, cs) {

    }

    void startStopwatch() {
      end_ts = millis() + max_ms;
      display_ts = millis();
      display_update_state = 0;
    }

    void setMaxMinutes( unsigned long minutes ) {
      max_ms = minutes * 60 * 1000;// convert to ms
    }

    void reset() {
      disableUSB();
      this->clear();
      enableUSB();
    }

    void showDone() {
      this->gotoXY(0, 1);
      this->print("- Done -");
    }

    // Non-blocking: spreads display update across multiple loop iterations.
    // Each call does at most ONE SPI operation to avoid blocking PID/odometry.
    bool timeRemaining() {
      unsigned long now = millis();
      bool remaining = (now < end_ts);

      // Idle: only start update when interval has elapsed
      if (display_update_state == 0) {
        if (now - display_ts <= display_interval_ms) {
          return remaining;
        }
        unsigned long dt = (end_ts - now) / 1000;
        // Clear only when transitioning to 2-digit (t=99) or 1-digit (t=9) to avoid artifacts
        display_update_state = (remaining && (dt == 99 || dt == 9)) ? 1 : 2;
      }

      // State 1: clear (only at t=99 and t=9)
      if (display_update_state == 1) {
        unsigned long t0 = micros();
        this->clear();
        Serial.print("OLED clear: ");
        Serial.print(micros() - t0);
        Serial.println(" us");
        display_update_state = 2;
        return remaining;
      }
      // State 2: gotoXY
      if (display_update_state == 2) {
        unsigned long t0 = micros();
        this->gotoXY(0, 1);
        Serial.print("OLED gotoXY: ");
        Serial.print(micros() - t0);
        Serial.println(" us");
        display_update_state = 3;
        return remaining;
      }
      // State 3: print
      if (display_update_state == 3) {
        unsigned long t0 = micros();
        if (remaining) {
          unsigned long dt = (end_ts - now) / 1000;
          this->print(dt);
        } else {
          this->print("- Done -");
        }
        Serial.print("OLED print: ");
        Serial.print(micros() - t0);
        Serial.println(" us");
        display_update_state = 0;
        display_ts = now;
        return remaining;
      }
      return remaining;
    }
  private:

    // Some variables to track time.
    unsigned long end_ts;
    unsigned long display_ts;
    unsigned long max_ms = 120000;
    // Non-blocking display: 0=idle, 1=clear, 2=gotoXY, 3=print
    uint8_t display_update_state = 0;
    static const unsigned long display_interval_ms = 1000;  // Update every 2s (was 1s) to reduce PID disruption

    // Two helper functions, disableUSB() and enableUSB().
    // Adapted from:
    // https://github.com/pololu/usb-pause-arduino/blob/master/USBPause.h
    // Accessed 25/09/24.
    uint8_t savedUDIEN;
    uint8_t savedUENUM;
    uint8_t savedUEIENX0;
    void disableUSB() {
      // Disable the general USB interrupt.  This must be done
      // first, because the general USB interrupt might change the
      // state of the EP0 interrupt, but not the other way around.
      savedUDIEN = UDIEN;
      UDIEN = 0;

      // Select endpoint 0.
      savedUENUM = UENUM;
      UENUM = 0;

      // Disable endpoint 0 interrupts.
      savedUEIENX0 = UEIENX;
      UEIENX = 0;
    }
    void enableUSB() {
      // Restore endpoint 0 interrupts.
      UENUM = 0;
      UEIENX = savedUEIENX0;

      // Restore endpoint selection.
      UENUM = savedUENUM;

      // Restore general device interrupt.
      UDIEN = savedUDIEN;
    }
};

#endif
