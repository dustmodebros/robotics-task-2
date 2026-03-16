#include "PID.h"
#include "Tuning.h"

#ifndef DEMAND_H
#define DEMAND_H

#define PID_UPDATE_MS 50
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