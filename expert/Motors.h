
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
#define L_PWM 10 // This is correct.
#define L_DIR 16 // This is correct.
#define R_PWM 9 // This is correct.
#define R_DIR 15 // This is correct.

// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MAX_PWM 180

#ifndef PID_UPDATE_MS
#define PID_UPDATE_MS 50
#endif

// Class to operate the motors.
class Motors_c {
  private:
    bool is_stopped;

  public:
    Motors_c() {}

    void initialise() {
      pinMode( L_PWM , OUTPUT );
      pinMode( L_DIR , OUTPUT );
      pinMode( R_PWM , OUTPUT );
      pinMode( R_DIR , OUTPUT );

      digitalWrite( L_DIR, HIGH );
      digitalWrite( R_DIR, HIGH );

      analogWrite( L_PWM , 0 );
      analogWrite( R_PWM , 0 );
    }

    bool checkMoving(unsigned long stop_moving_at) {
      if (!is_stopped && millis() > stop_moving_at) {
        setPWM(0, 0);
        is_stopped = true;
      }
      return is_stopped;
    }

    void setPWM( float left_pwr, float right_pwr ) {
      if (abs(left_pwr) > 1 or abs(right_pwr) > 1){
        analogWrite( L_PWM , 0);
        analogWrite( R_PWM , 0);
        return;
      }
      analogWrite( L_PWM , abs(left_pwr) * MAX_PWM);
      analogWrite( R_PWM , abs(right_pwr) * MAX_PWM);
      if ( left_pwr <= 0 ) {
        digitalWrite( L_DIR, HIGH );
      } else {
        digitalWrite( L_DIR, LOW );
      }
      if ( right_pwr <= 0 ) {
        digitalWrite( R_DIR, HIGH );
      } else {
        digitalWrite( R_DIR, LOW );
      }
      return;
    }

    void stopAfterTurn(float left_bias, float right_bias, float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
      setPWM(left_bias, right_bias);
      is_stopped = false;
    }
}; // End of Motors_c class definition.

#endif
