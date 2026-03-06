/***************************************
   ,        .       .           .      , 
   |        |       |           |     '| 
   |    ,-: |-. ,-. |-. ,-. ,-. |-     | 
   |    | | | | `-. | | |-' |-' |      | 
   `--' `-` `-' `-' ' ' `-' `-' `-'    ' 
 A D V A N C E D   E X E R C I S E S
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler. 

#include "Tuning.h"

#ifndef _PID_H
#define _PID_H

#define MAX_TURN_PWM 0.4  // max wheel speed during turn-in-place to prevent skidding (tune if needed)
#define TURN_IN_PLACE_THRESHOLD (PI/4)  // angle error above which we turn in place (opposite wheels)
#define POSITION_TOLERANCE 90

// PID TUNING
const extern float left_p;
const extern float left_i;
const extern float left_d;
const extern float right_p;
const extern float right_i;
const extern float right_d;
const extern float heading_p;
const extern float heading_i;
const extern float heading_d;

// Class to contain generic PID algorithm.
class PID_c {
  public:

    // PID update variables.
    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    // To store gains.
    float p_gain;
    float i_gain;
    float d_gain;

    // To determine time elapsed.
    unsigned long ms_last_t;
  
    // Constructor, must exist.
    PID_c() {
      // leaving this empty.
      // ensure initialise() is called
      // instead.
    } 

    // To setup PID.
    // Pass in gain values for kp, ki, kd.
    void initialise( float p, float i, float d ) {
       feedback = 0;
       last_error = 0;
       p_term = 0;
       i_term = 0;
       d_term = 0;
       i_sum = 0;

       p_gain = p;
       i_gain = i;
       d_gain = d;

       ms_last_t = millis();
    }

    // If PID hasn't been called in a long time,
    // the time elapsed will be huge and cause a
    // massive overshoot.  It is best to reset if
    // we know the robot has been delayed or motors
    // disabled.
    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;
      ms_last_t = millis();
    }

    // To imlpement this on a microcontroller, this is a good
    // referenece:
    // https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
    float update( float demand, float measurement ) {
      float error;
      unsigned long ms_now_t;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;

      // Grab time to calc elapsed time.
      ms_now_t = millis();
      ms_dt = ms_now_t - ms_last_t;

      // ms_last_t has been used, so update
      // it for the next call of this update.
      ms_last_t = millis();
      
      // typecasting the different of two
      // unsigned longs is safer.
      float_dt = (float)ms_dt;

      // NOTE: A serious error can occur
      // here if dt is 0, this causes divide
      // by zero errors.  This can happen if
      // PID.update() is called faster than 1ms.
      // Here, we catch the error by returning
      // the last feedback value.
      if( float_dt == 0.0 ) return feedback;

      // Calculate error signal.
      error = demand - measurement;

      // Update 03/10/25: Commented out here,
      // this needs to happen after d-term.
      //last_error = error;

      // P term, nice and easy
      p_term = p_gain * error;

      // discrete integration
      i_sum = i_sum + (error * float_dt);

      // i_term.
      i_term = i_gain * i_sum;
      
      // d_term.
      // Note, sometimes this needs to be inverted
      // to last_error - error.  It depends on the
      // error signal sign in relation to the system.
      diff_error = (error - last_error) / float_dt;
      d_term = diff_error * d_gain;

      // Update 03/10/25.  Added line here, 
      // fixing minor d-term bug.
      last_error = error;
	

      // Sometimes this needs to be - d_term.
      // d_term should counteract sudden changes, which
      // means it is subtractive.  Another way to achieve
      // this is to have a negative gain.  The best way 
      // to troubleshoot is to plot what it is doing.
      feedback = p_term + i_term + d_term;

      // done
      return feedback;
    }

}; // End of PID_c class definition

class Demand {
  private:
    PID_c left_pid;
    PID_c right_pid;
    PID_c heading;

    bool is_stopped_demand;
    unsigned long stop_moving_at_demand;

    float left_demand; // requested wheel speed
    float right_demand; // requested wheel speed
    unsigned long pid_update_ts;  // timestamp for updating PID values


    float sign(float x) {
      return (x > 0) ? 1.0f : -1.0f;
    }

  public:
    bool enable_demand;
    unsigned long enable_demand_ms; // wait 5s before enabling demand
    unsigned long enable_demand_ts;
    unsigned long stop_moving_at;

    Demand() {}

    void initialise() {
      left_pid.initialise(left_p, left_i, left_d); // tuned values
      right_pid.initialise(right_p, right_i, right_d);
      heading.initialise(heading_p, heading_i, heading_d);
      enable_demand = false;
      enable_demand_ms = 5000; // wait 5s before enabling demand
      pid_update_ts = millis();
      resetPIDs();
    }

    void resetPIDs() {
      left_pid.reset();
      right_pid.reset();
      heading.reset();
    }

    float updateLeft(float demand, float smoothed_speed) {
      return left_pid.update(demand, smoothed_speed);
    }

    float updateRight(float demand, float smoothed_speed) {
      return right_pid.update(demand, smoothed_speed);
    }

    float updateHeading(float angle_diff) {
      return heading.update(0, angle_diff);
    }

    bool checkMovingDemand() {
      if (!is_stopped_demand && millis() > stop_moving_at_demand) {
        zeroDemand();
        is_stopped_demand = true;
      }
      return !is_stopped_demand;
    }

    void zeroDemand() {
      left_demand = 0;
      right_demand = 0;
    }

    bool calculatePWMs(float *l_pwm, float *r_pwm, float l_speed, float r_speed) {
      const unsigned long now = millis();
      if (now - pid_update_ts <= PID_UPDATE_MS) {return false;}
      pid_update_ts = now;
      *l_pwm = updateLeft(left_demand, l_speed);
      *r_pwm = updateRight(right_demand, r_speed);
      return true;
    }

    void checkEnableDemand() {
      // Wait for demand to be enabled after initialisation delay
      if (!enable_demand && millis() > enable_demand_ts + enable_demand_ms) {
        enable_demand = true;
        resetPIDs();
      }
    }

    bool doCheckTravel(float diff, float stop_distance_mm, float dist_sq) {
      const float threshold_sq = (stop_distance_mm > 0) ? (stop_distance_mm * stop_distance_mm) : POSITION_TOLERANCE;
      if (dist_sq > threshold_sq) {
        const float turn_scaling_factor = 0.5;
        if (abs(diff) > TURN_IN_PLACE_THRESHOLD) {
          // Large angle error: turn in place with opposite wheel demands for precise turning.
          // Limit wheel speed to prevent skidding.
          const float turn_speed = MAX_TURN_PWM * sign(diff);
          left_demand = -turn_speed;
          right_demand = turn_speed;
        } else {
          // Small angle error: drive forward with turn correction
          float turn_amount = diff * turn_scaling_factor;
          if (turn_amount > MAX_TURN_PWM) turn_amount = MAX_TURN_PWM;
          if (turn_amount < -MAX_TURN_PWM) turn_amount = -MAX_TURN_PWM;
          left_demand = 0.3 - turn_amount;
          right_demand = 0.3 + turn_amount;
        }
        return false;
      } else {
        zeroDemand();
        return true;
      }
    }
};

#endif
