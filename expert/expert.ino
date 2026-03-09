#include <Wire.h>
#include <LIS3MDL.h>
#include "Motors.h"
#include "Demand.h"
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "Waypoints.h"

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
Speed speed;
Waypoints waypoints;
Demand demand;

// --- GLOBAL VARIABLES ---

// BUZZER
#define BUZZER_PIN 6

// DEBUG PRINTS
unsigned long debug_ts = millis();
unsigned long debug_ms = 500;

// CLOSED-LOOP CONTROL
float target_angle = 0;
#define TURNING_SENSITIVITY PI/256 //when to stop turning
float target_x;
float target_y;
unsigned long travel_ts = 0;  // 0 ensures first checkTravel runs immediately
unsigned long travel_ms = 50;

// WAYPOINTS
#define CUP_AVOIDANCE_AMOUNT 150

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


// stop_distance_mm: when > 0, stop when within this many mm of target. When <= 0, use POSITION_TOLERANCE.
// returns affirmative if we've reached our destination
bool checkTravel(float stop_distance_mm = -1) {
  const unsigned long now = millis();
  if (now < travel_ts + travel_ms) {
    return false;  // Rate limit: don't update demands yet
  }
  travel_ts = now;
  const float dist_sq = pose.distSq(target_x, target_y); // distance to the stopping point);
  const float diff = pose.angleDiff(target_x, target_y);
  return demand.doCheckTravel(diff, stop_distance_mm, dist_sq);
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
      float demand_turn_speed = demand.updateHeading(angle_diff);
  
      // Generate the required PWM signal based
      // on a speed demand and measurement.
      float l_pwm = demand.updateLeft(demand_turn_speed, speed.smoothed_speed_left );
      float r_pwm = demand.updateRight(-demand_turn_speed, speed.smoothed_speed_right );
      // Send the PWM output to the left and right motor
      motors.setPWM(l_pwm, r_pwm);
    }
      // Signal that the turn is still in progress
      // by returning "false"
      return false;
  } else {
    // Turn finished, stop the robot.
    motors.setPWM(0, 0);
    demand.enable_demand = true;
    // Signal that the turn finished by
    // returning "true".
    return true;
  }
}

void checkDemand() {
  if (!demand.enable_demand) {return;}
  obeyDemand(); // Set motor PWM according to demand
}

void doFoundCup() {
  current_state = FOUND_CUP;
  demand.enable_demand_ts = millis() - demand.enable_demand_ms + 500;
  demand.enable_demand = false;
  motors.setPWM(0,0);
}

void setTurn(float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
  float left_bias, right_bias;
  computeTurnBias(fwd_bias_pwm, turn_pwm, left_bias, right_bias);
  demand.stop_moving_at = millis() + duration_ms;
  motors.stopAfterTurn(left_bias, right_bias, fwd_bias_pwm, turn_pwm, duration_ms);
}

void obeyDemand() {
  float l_pwm, r_pwm;
  if (!demand.calculatePWMs(&l_pwm, &r_pwm, speed.smoothed_speed_left, speed.smoothed_speed_right)) {return;}
  motors.setPWM(l_pwm, r_pwm);
}

void doSearch() {
  if (checkTravel()){
    waypoints.increment();
    setTravel(waypoints.current_x(), waypoints.current_x());
  }
  if (magnetometer.convertToMagnitude() > 2.7){
    setTurn(-0.2,0,400);
    current_state = BACKING_UP;
  }
}

void doBackingUp() {
  if (motors.checkMoving(demand.stop_moving_at)){
    current_state = GOTO_BEHIND_CUP;
  }
}

void doGotoBehindCup() {
  float cup_location_x = waypoints.current_x();
  float cup_location_y = waypoints.current_y();
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
  setTravel(0, waypoints.origin_y);
  if (checkTravel(90)) {  // Stop when within 90mm of origin to avoid pushing cup out of end zone
    now = millis();
    current_state = WAIT_FOR_RESET;
  }
}

void doWaitForReset() {
  if (now + 4000 >= millis()){return;}
  current_state = SEARCH;
  waypoints.current = 0;
}

void doFinished() {
  analogWrite(6, HIGH);
  while(1){
    demand.zeroDemand();
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

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  Wire.begin();

  // Activates the Serial port, and the delay
  // is used to wait for the connection to be
  // established.
  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");

  // If you have a problem with your magnetometer, your code
  // will get stuck here and print the below message.
  motors.initialise();
  line_sensors.initialiseForADC();
  magnetometer.initialise();
  demand.initialise();
  speed.initialise();
  pose.initialise(0, 0, 0);
  waypoints.initialise();

  current_state = SEARCH;

  calibrateSensors();

  stop_ts = millis() + 240000; // stop after four minutes
}

void loop() {
  checkStop();
  demand.checkEnableDemand();  
  (void)motors.checkMoving(demand.stop_moving_at);
  speed.computeSpeed(); // update global variables with new speed estimates
  checkDemand();
  pose.checkUpdate(); // update kinematics
  magnetometer.doCalibratedReadings();
  waypoints.doDrift();
  checkState();
}
