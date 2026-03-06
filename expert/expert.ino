#include <Wire.h>
#include <LIS3MDL.h>
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Kinematics.h"
#include "Encoders.h"

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
Speed speed;

// --- GLOBAL VARIABLES ---

// BUZZER
#define BUZZER_PIN 6

// DEMAND
bool is_stopped_demand;
bool enable_demand;
unsigned long enable_demand_ts;
unsigned long enable_demand_ms; // wait 5s before enabling demand
unsigned long stop_moving_at;
unsigned long stop_moving_at_demand;

// PID TUNING
PID_c left_pid;
PID_c right_pid;
float left_demand; // requested wheel speed
float right_demand; // requested wheel speed
unsigned long pid_update_ts;  // timestamp for updating PID values
#define PID_UPDATE_MS 50     // periodicity of PID update

// DEBUG PRINTS
unsigned long debug_ts = millis();
unsigned long debug_ms = 500;

// CLOSED-LOOP CONTROL
float target_angle = 0;
#define TURNING_SENSITIVITY PI/256 //when to stop turning
#define POSITION_TOLERANCE 90
#define MAX_TURN_PWM 0.4  // max wheel speed during turn-in-place to prevent skidding (tune if needed)
#define TURN_IN_PLACE_THRESHOLD (PI/4)  // angle error above which we turn in place (opposite wheels)
PID_c heading;
float target_x;
float target_y;
unsigned long travel_ts = 0;  // 0 ensures first checkTravel runs immediately
unsigned long travel_ms = 50;

// WAYPOINTS
#define NUM_WAYPOINTS 7
#define CUP_AVOIDANCE_AMOUNT 150
float waypoints_x[NUM_WAYPOINTS] = {0, 69.83, 266.69, 432.97, 411.76, 355.68, 162.42};
float waypoints_y[NUM_WAYPOINTS] = {0, 269.21, 215.27, 264.17, 102.55, -18.66, 85.01}; //1, 2, 3, 4, 5 and 6
int current_waypoint = 0;
float origin_y = 0;  // Drifted with waypoints
unsigned long waypoint_drift_ts = 0;
#define WAYPOINT_DRIFT_RATE_MM_S (- 5.0f / 24.0f)

// STOP TIMER
unsigned long stop_ts;

// FSM
typedef enum{
  SEARCH,
  FOUND_CUP,
  BACKING_UP,
  GOTO_BEHIND_CUP,
  DETOUR,
  STRAIGHT_TO_BEHIND,
  POINT_AT_ORIGIN,
  GOTO_ORIGIN,
  WAIT_FOR_RESET,
  FINISHED,
  DEBUG = 413
} state;

state current_state;
float alignment_target_x;
float alignment_target_y; //the point behind the cup
float pathway_x;
float pathway_y; //the perpendicular point
unsigned long now;


// Computes left/right wheel PWM from forward bias and turn amount.
// Positive turn_pwm = turn right, negative = turn left.
void computeTurnBias(float fwd_bias_pwm, float turn_pwm, float& left_bias, float& right_bias) {
  float turn_amount = turn_pwm * fwd_bias_pwm;
  float remove_from_right = (turn_pwm > 0) ? abs(turn_amount) : 0;
  float remove_from_left = (turn_pwm <= 0) ? abs(turn_amount) : 0;
  right_bias = fwd_bias_pwm - 2 * remove_from_right;
  left_bias = fwd_bias_pwm - 2 * remove_from_left;
}

void setTravel(float x, float y) {
  target_x = x;
  target_y = y;
}

float sign(float x) {
  return (x > 0) ? 1.0f : -1.0f;
}

// stop_distance_mm: when > 0, stop when within this many mm of target. When <= 0, use POSITION_TOLERANCE.
// returns affirmative if we've reached our destination
bool checkTravel(float stop_distance_mm = -1) {
  const unsigned long now = millis();
  if (now < travel_ts + travel_ms) {
    return false;  // Rate limit: don't update demands yet
  }
  travel_ts = now;

  const float dist_sq = pose.distSq(target_x, target_y); // distance to the stopping point
  const float threshold_sq = (stop_distance_mm > 0) ? (stop_distance_mm * stop_distance_mm) : POSITION_TOLERANCE;
  if (dist_sq > threshold_sq) {
    const float diff = pose.angleDiff(target_x, target_y);
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
    left_demand = 0;
    right_demand = 0;
    return true;
  }
}

void calibrateSensors() {
  const unsigned long start_time = millis();
  const unsigned long duration = 2400;

  while (millis() - start_time < duration) {
    setTurn(0.175, 1, 100); // rotate clockwise for 100 milliseconds

    // Line sensor calibration
    line_sensors.calibrate();

    // Magnetometer calibration
    magnetometer.calibrate();
    delay(100);
  }
}

// A non-blocking function to rotate the robot
// until a desired kinematics theta angle is
// achieved.
bool checkTurn() {
  // Have we finished the turn? If not:
  float angle_diff = pose.smallestAngle(target_angle);
  const float turn_gain = 1;
  if(abs(angle_diff) > TURNING_SENSITIVITY) {
    if (millis() > travel_ts + travel_ms){
      angle_diff = angle_diff * turn_gain;
  
      // Generate a speed demand for rotation
      // based on a demand and measured theta
      // angle from kinematics.
      // Note, the following approach will minimise
      // the angular difference between the current
      // theta in kinematics and the desired angle.
      float demand_turn_speed = heading.update( 0, angle_diff );
  
      // Generate the required PWM signal based
      // on a speed demand and measurement.
      float l_pwm = left_pid.update( demand_turn_speed, speed.smoothed_speed_left );
      float r_pwm = right_pid.update( -demand_turn_speed, speed.smoothed_speed_right );

      // Send the PWM output to the left and right motor
      motors.setPWM(l_pwm, r_pwm);
    }
      // Signal that the turn is still in progress
      // by returning "false"
      return false;
  } else {
    // Turn finished, stop the robot.
    motors.setPWM(0, 0);
    enable_demand = true;
    // Signal that the turn finished by
    // returning "true".
    return true;
  }
}

void checkDemand() {
  if (!enable_demand) {return;}
  obeyDemand(); // Set motor PWM according to demand
}

void doFoundCup() {
  current_state = FOUND_CUP;
  enable_demand_ts = millis() - enable_demand_ms + 500;
  enable_demand = false;
  motors.setPWM(0,0);
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

void setTurn(float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
  float left_bias, right_bias;
  computeTurnBias(fwd_bias_pwm, turn_pwm, left_bias, right_bias);
  stop_moving_at = millis() + duration_ms;
  motors.stopAfterTurn(left_bias, right_bias, fwd_bias_pwm, turn_pwm, duration_ms);
}

void obeyDemand() {
  const unsigned long now = millis();
  if (now - pid_update_ts <= PID_UPDATE_MS) {return;}
  pid_update_ts = now;
  const float l_pwm = left_pid.update(left_demand, speed.smoothed_speed_left);
  const float r_pwm = right_pid.update(right_demand, speed.smoothed_speed_right);
  motors.setPWM(l_pwm, r_pwm);
}

bool checkMovingDemand() {
  if (!is_stopped_demand && millis() > stop_moving_at_demand) {
    left_demand = 0;
    right_demand = 0;
    is_stopped_demand = true;
  }
  return !is_stopped_demand;
}

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  Wire.begin();
  motors.initialise(); // begin motor control
  line_sensors.initialiseForADC();
  setupEncoder0();
  setupEncoder1();

  speed.initialise();

  pose.initialise(0, 0, 0);

  current_state = SEARCH;

  // Activates the Serial port, and the delay
  // is used to wait for the connection to be
  // established.
  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");

  // If you have a problem with your magnetometer, your code
  // will get stuck here and print the below message.
  magnetometer.initialise();

  enable_demand = false;
  enable_demand_ms = 5000; // wait 5s before enabling demand

  left_pid.initialise(0.3, 0.001, 0.0); // tuned values
  right_pid.initialise(0.31, 0.001, 0.0);
  heading.initialise(0.7, 0.0, 0.0); 

  pid_update_ts = millis();

  left_pid.reset(); // needed because we delay to wait for the serial connection, that phutzes with timing
  right_pid.reset();

  calibrateSensors();

  stop_ts = millis() + 240000; // stop after four minutes
}


void doSearch() {
  if (checkTravel()){
    current_waypoint += 1;
    setTravel(waypoints_x[current_waypoint % 7], waypoints_y[current_waypoint % 7]);
  }
  if (magnetometer.convertToMagnitude() > 2.7){
    setTurn(-0.2,0,400);
    current_state = BACKING_UP;
  }
}

void doBackingUp() {
  if (motors.checkMoving(stop_moving_at)){
    current_state = GOTO_BEHIND_CUP;
  }
}

void doGotoBehindCup() {
  float cup_location_x = waypoints_x[current_waypoint % 7];
  float cup_location_y = waypoints_y[current_waypoint % 7];
  float cup_theta = atan2(cup_location_y,cup_location_x);
  alignment_target_x = cup_location_x + cos(cup_theta) * CUP_AVOIDANCE_AMOUNT;
  alignment_target_y = cup_location_y + sin(cup_theta) * CUP_AVOIDANCE_AMOUNT;
  float potential_x_1 = cup_location_x - sin(cup_theta) * CUP_AVOIDANCE_AMOUNT;
  float potential_x_2 = cup_location_x + sin(cup_theta) * CUP_AVOIDANCE_AMOUNT;
  float potential_y_1 = cup_location_y + cos(cup_theta) * CUP_AVOIDANCE_AMOUNT;
  float potential_y_2 = cup_location_y - cos(cup_theta) * CUP_AVOIDANCE_AMOUNT;

  // pathway waypoint will be whichever of the potentials is closest to the current pose x and y
  float distance_target = pose.dist(alignment_target_x, alignment_target_y);
  float distance_potential_1 = pose.dist(potential_x_1, potential_y_1);
  float distance_potential_2 = pose.dist(potential_x_2, potential_y_2);
  if (distance_target < distance_potential_1 and distance_target < distance_potential_2){
//  Serial.println("Going straight to the alignment point");
    current_state = STRAIGHT_TO_BEHIND;
  } else {
    if (distance_potential_1 < distance_potential_2){
//    Serial.println("Going through alignment point 1");
      pathway_x = potential_x_1;
      pathway_y = potential_y_1;
     } else {
//      Serial.println("Going through alignment point 2");
        pathway_x = potential_x_2;
        pathway_y = potential_y_2;
      }
    current_state = DETOUR;
  }
}

void doDetour() {
  setTravel(pathway_x, pathway_y);
  if(checkTravel()){
    current_state = STRAIGHT_TO_BEHIND;
  }
}

void doStraightToBehind() {
  setTravel(alignment_target_x, alignment_target_y);
  if(checkTravel()){
    current_state = POINT_AT_ORIGIN;
  }
}

void doPointAtOrigin() {
  target_angle = pose.angleToOrigin();
  if(checkTurn()){
    current_state = GOTO_ORIGIN;
  }
}

void doGotoOrigin() {
  setTravel(0, origin_y);
  if (checkTravel(90)) {  // Stop when within 90mm of origin to avoid pushing cup out of end zone
    now = millis();
    current_state = WAIT_FOR_RESET;
  }
}

void doWaitForReset() {
  if (now + 4000 >= millis()){return;}
  current_state = SEARCH;
  current_waypoint = 0;
}

void doFinished() {
  analogWrite(6, HIGH);
  while(1){
    left_demand = 0;
    right_demand = 0;
    motors.setPWM(0,0);
    delay(100);
  }
}

void doDebug() {
  if (millis() <= debug_ts + debug_ms){return;}
  debug_ts = millis();
  // Serial.print(pose.x);
  // Serial.print(", ");
  // Serial.print(pose.y);
  // Serial.print(", ");
  // Serial.print(pose.theta);
  // Serial.print(",           ");
  Serial.print(target_x);
  Serial.print(",");
  Serial.println(target_y);
}

void checkState() {
  switch (current_state){
    case SEARCH:
      doSearch();
      break;
    case FOUND_CUP:
      doFoundCup();
      break;
    case BACKING_UP:
      doBackingUp();
      break;
    case GOTO_BEHIND_CUP:
      doGotoBehindCup();
      break;
    case DETOUR:
      doDetour();
      break;
    case STRAIGHT_TO_BEHIND:
      doStraightToBehind();
      break;
    case POINT_AT_ORIGIN:
      doPointAtOrigin();
      break;
    case GOTO_ORIGIN:
      doGotoOrigin();
      break;
    case WAIT_FOR_RESET:
      doWaitForReset();
      break;
    case FINISHED:
      doFinished();
      break;
    case DEBUG:
      doDebug();
      break;
  }
}

void checkStop() {
  if (millis() < stop_ts) {return;}
  current_state = FINISHED;
}

void driftWaypoints() {
  // Drift waypoints' and origin's y coordinates by -5/24 mm every second
  if (millis() - waypoint_drift_ts >= 1000) {
    waypoint_drift_ts = millis();
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      waypoints_y[i] -= WAYPOINT_DRIFT_RATE_MM_S;
    }
    origin_y -= WAYPOINT_DRIFT_RATE_MM_S;
  }
}

void loop() {
  checkStop();
  checkEnableDemand();  
  (void)motors.checkMoving(stop_moving_at);
  speed.computeSpeed(); // update global variables with new speed estimates
  checkDemand();
  pose.checkUpdate(); // update kinematics
  driftWaypoints();
  magnetometer.doCalibratedReadings();
  checkState();
}
