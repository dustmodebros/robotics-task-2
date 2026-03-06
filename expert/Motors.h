
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
    unsigned long stop_moving_at;
    unsigned long stop_moving_at_demand;
    bool is_stopped_demand;
    bool enable_demand;

    unsigned long enable_demand_ts;
    unsigned long enable_demand_ms; // wait 5s before enabling demand

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

      enable_demand = false;
      enable_demand_ms = 5000; // wait 5s before enabling demand
    }

    void setTurn(float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
      float left_bias, right_bias;
      computeTurnBias(fwd_bias_pwm, turn_pwm, left_bias, right_bias);
      stop_moving_at = millis() + duration_ms;
      setPWM(left_bias, right_bias);
      is_stopped = false;
    }

    void doFinished() {
      analogWrite(6, HIGH);
      while(1){
        left_demand = 0;
        right_demand = 0;
        setPWM(0,0);
        delay(100);
      }
    }

    void obeyDemand() {
      const unsigned long now = millis();
      if (now - pid_update_ts <= PID_UPDATE_MS) {return;}
      pid_update_ts = now;
      const float l_pwm = left_pid.update(left_demand, smoothed_speed_left);
      const float r_pwm = right_pid.update(right_demand, smoothed_speed_right);
      setPWM(l_pwm, r_pwm);
    }

    bool checkMovingDemand() {
      if (!is_stopped_demand && millis() > stop_moving_at_demand) {
        left_demand = 0;
        right_demand = 0;
        is_stopped_demand = true;
      }
      return !is_stopped_demand;
    }

    bool checkMoving() {
      if (!is_stopped && millis() > stop_moving_at) {
        setPWM(0, 0);
        is_stopped = true;
      }
      return is_stopped;
    }

    void checkDemand() {
      if (!enable_demand) {return;}
      obeyDemand(); // Set motor PWM according to demand
    }

    void checkEnableDemand() {
      // Wait for demand to be enabled after initialisation delay
      if (!enable_demand && millis() > enable_demand_ts + enable_demand_ms) {
        enable_demand = true;
        left_pid.reset();
        right_pid.reset();
        heading.reset();
      }
    }

    void doFoundCup() {
      current_state = FOUND_CUP;
      enable_demand_ts = millis() - enable_demand_ms + 500;
      enable_demand = false;
      setPWM(0,0);
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

    void stopRobot() {
      motors.setPWM(0, 0);
      enable_demand = true;
    }
}; // End of Motors_c class definition.



#endif
