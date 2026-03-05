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
LIS3MDL mag;
Kinematics_c pose;

// --- GLOBAL VARIABLES ---

// BUZZER
#define BUZZER_PIN 6

// MOVEMENT CONTROL
unsigned long stop_moving_at;
unsigned long stop_moving_at_demand;
bool is_stopped;
bool is_stopped_demand;
bool enable_demand = false;

unsigned long enable_demand_ts;
unsigned long enable_demand_ms = 5000; // wait 5s before enabling demand

// SPEED ESTIMATION
#define SPEED_EST_MS 10     // estimate speed every 10ms
unsigned long speed_est_ts;   // timestamp for speed estimation
long last_e0; // previous count of pulses from encoder 0
float speed_left; // Wheel speed as a global float
long last_e1; // last right encoder pulses
float speed_right; // right wheel speed

// SPEED SMOOTHING
#define SMOOTHING_FACTOR 0.7  // factor for smoothing of speed estimation
float smoothed_speed_left;
float last_smoothed_speed_left;
float smoothed_speed_right;
float last_smoothed_speed_right;

// PID TUNING
PID_c left_pid;
PID_c right_pid;
float left_demand; // requested wheel speed
float right_demand; // requested wheel speed
unsigned long pid_update_ts;  // timestamp for updating PID values
#define PID_UPDATE_MS 50     // periodicity of PID update

// IR SENSOR TUNING
float max_ir_readings[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float min_ir_readings[] = {1024.0, 1024.0, 1024.0, 1024.0, 1024.0};

// MAGNET CALIBRATION
float max_mag_readings[] = { -9999.0, -9999.0, -9999.0};
float min_mag_readings[] = {9999.0, 9999.0, 9999.0};
float calibrated_mag[3];

// DEBUG PRINTS
unsigned long debug_ts = millis();
unsigned long debug_ms = 500;

// KINEMATICS TUNING
unsigned long pose_update_ts = millis();
unsigned long pose_update_ms = 30;

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

void setTurnDemand(float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
  float left_bias, right_bias;
  computeTurnBias(fwd_bias_pwm, turn_pwm, left_bias, right_bias);
  stop_moving_at_demand = millis() + duration_ms;
  left_demand = left_bias;
  right_demand = right_bias;
  is_stopped_demand = false;
}

void setTurn(float fwd_bias_pwm, float turn_pwm, unsigned long duration_ms) {
  float left_bias, right_bias;
  computeTurnBias(fwd_bias_pwm, turn_pwm, left_bias, right_bias);
  stop_moving_at = millis() + duration_ms;
  motors.setPWM(left_bias, right_bias);
  is_stopped = false;
}

bool checkMoving() {
  if (!is_stopped && millis() > stop_moving_at) {
    motors.setPWM(0, 0);
    is_stopped = true;
  }
  return is_stopped;
}

bool checkMovingDemand() {
  if (!is_stopped_demand && millis() > stop_moving_at_demand) {
    left_demand = 0;
    right_demand = 0;
    is_stopped_demand = true;
  }
  return !is_stopped_demand;
}

void computeSpeed() {
  unsigned long elapsed_time = millis() - speed_est_ts;
  if ( elapsed_time > SPEED_EST_MS) {
    speed_est_ts = millis();

    // Work out the difference in encoder counts
    long count_difference_right = count_e0 - last_e0;
    long count_difference_left = count_e1 - last_e1;

    // Save the current count as the last count
    last_e0 = count_e0;
    last_e1 = count_e1;

    // turn counts into speed by dividing by timestep
    speed_left = count_difference_left / float(SPEED_EST_MS);
    speed_right = count_difference_right / float(SPEED_EST_MS);

    // compute smoothed speed
    smoothed_speed_left = (SMOOTHING_FACTOR * speed_left) + ((1 - SMOOTHING_FACTOR) * last_smoothed_speed_left);
    smoothed_speed_right = (SMOOTHING_FACTOR * speed_right) + ((1 - SMOOTHING_FACTOR) * last_smoothed_speed_right);

    // set current smoothed speed as previous for next loop
    last_smoothed_speed_left = smoothed_speed_left;
    last_smoothed_speed_right = smoothed_speed_right;
  }
}


void calcCalibratedMag(float min_values[3], float max_values[3]) {
  mag.read();
  float readings[3] = {(float)mag.m.x, (float)mag.m.y, (float)mag.m.z};
  for (int i = 0; i < 3; i++) {
    calibrated_mag[i] = 2.0f * (readings[i] - min_values[i]) / (max_values[i] - min_values[i]) - 1.0f;
  }
}

void obeyDemand() {
  unsigned long now = millis();
  if (now - pid_update_ts > PID_UPDATE_MS) {
    pid_update_ts = now;
    float l_pwm = left_pid.update(left_demand, smoothed_speed_left);
    float r_pwm = right_pid.update(right_demand, smoothed_speed_right);
    motors.setPWM(l_pwm, r_pwm);
  }
}

float convertToMagnitude() {
  float sum = 0;
  for (int i = 0; i < 3; i++) {
    sum += calibrated_mag[i] * calibrated_mag[i];
  }
  return sqrt(sum);
}

void setTravel(float x, float y) {
  target_x = x;
  target_y = y;
}

float sign(float x) {
  return (x > 0) ? 1.0f : -1.0f;
}

// stop_distance_mm: when > 0, stop when within this many mm of target. When <= 0, use POSITION_TOLERANCE.
bool checkTravel(float stop_distance_mm = -1) {
  unsigned long now = millis();
  if (now < travel_ts + travel_ms) {
    return false;  // Rate limit: don't update demands yet
  }
  travel_ts = now;

  float dist_sq = pow(pose.x - target_x, 2) + pow(pose.y - target_y, 2); // distance to the stopping point
  float threshold_sq = (stop_distance_mm > 0) ? (stop_distance_mm * stop_distance_mm) : POSITION_TOLERANCE;
  if (dist_sq > threshold_sq) {
    float theta_d = atan2(target_y-pose.y, target_x-pose.x); // desired angle
    float diff = smallestAngle(pose.theta, theta_d);
    float turn_scaling_factor = 0.5;

    if (abs(diff) > TURN_IN_PLACE_THRESHOLD) {
      // Large angle error: turn in place with opposite wheel demands for precise turning.
      // Limit wheel speed to prevent skidding.
      float turn_speed = MAX_TURN_PWM * sign(diff);
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
  unsigned long start_time = millis();
  unsigned long duration = 2400;

  while ( millis() - start_time < duration) {
    setTurn(0.175, 1, 100); // rotate clockwise for 100 milliseconds

    // Line sensor calibration
    line_sensors.readSensorsADC();
    for (int i = 0; i < 5; i++) {
      max_ir_readings[i] = max(line_sensors.readings[i], max_ir_readings[i]);
      min_ir_readings[i] = min(line_sensors.readings[i], min_ir_readings[i]);
    }

    // Magnetometer calibration
    mag.read();
    float readings[3] = {(float)mag.m.x, (float)mag.m.y, (float)mag.m.z};
    for (int i = 0; i < 3; i++) {
      max_mag_readings[i] = max(readings[i], max_mag_readings[i]);
      min_mag_readings[i] = min(readings[i], min_mag_readings[i]);
    }
    delay(100);
  }
}

float smallestAngle(float theta_i, float desired_angle) {
  float diff = desired_angle - theta_i;
  while (diff > PI) diff -= 2.0f * PI;
  while (diff < -PI) diff += 2.0f * PI;
  return diff;
}

// A non-blocking function to rotate the robot
// until a desired kinematics theta angle is
// achieved.
bool checkTurn() {
  // Have we finished the turn? If not:
  float angle_diff = smallestAngle(pose.theta, target_angle);
  float turn_gain = 1;
  if( abs(angle_diff) > TURNING_SENSITIVITY) {
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
      float l_pwm = left_pid.update( demand_turn_speed, smoothed_speed_left );
      float r_pwm = right_pid.update( -demand_turn_speed, smoothed_speed_right );
  
      // Send the PWM output to the left and right motor
      motors.setPWM( l_pwm, r_pwm );
 
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

void updatePose(){
  unsigned long now = millis();
  if (now > pose_update_ts + pose_update_ms) {
    pose_update_ts = now;
    pose.update();
  }
}

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  Wire.begin();
  motors.initialise(); // begin motor control
  line_sensors.initialiseForADC();
  setupEncoder0();
  setupEncoder1();

  // This takes the initial count from encoder 0
  // and saves it as the "last" (previous) count
  last_e0 = count_e0;
  last_e1 = count_e1;

  // Assuming we start with motors off.
  speed_left = 0.0;
  speed_right = 0.0;

  speed_est_ts = millis(); // initialise timestamp for speed estimation

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
  if (!mag.init() ) {
    while (1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }
  mag.enableDefault();

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
  if (convertToMagnitude() > 2.7){
    current_state = FOUND_CUP;
    enable_demand_ts = millis() - enable_demand_ms + 500;
    enable_demand = false;
    motors.setPWM(0,0);
  }
}

void doFoundCup() {
  setTurn(-0.2,0,400);
  current_state = BACKING_UP;
}

void doBackingUp() {
  if (checkMoving()){
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
  float distance_target = sqrt(pow(alignment_target_x - pose.x, 2) + pow(alignment_target_y - pose.y, 2));
  float distance_potential_1 = sqrt(pow(potential_x_1 - pose.x, 2) + pow(potential_y_1 - pose.y, 2));
  float distance_potential_2 = sqrt(pow(potential_x_2 - pose.x, 2) + pow(potential_y_2 - pose.y, 2));
  if (distance_target < distance_potential_1 and distance_target < distance_potential_2){
//        Serial.println("Going straight to the alignment point");
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
  target_angle = atan2(-pose.x, -pose.y);
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
  if (now + 4000 < millis()){
    current_state = SEARCH;
    current_waypoint = 0;
  }
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
  if (millis() > debug_ts + debug_ms){
    debug_ts = millis();
    Serial.print(pose.x);
    Serial.print(", ");
    Serial.print(pose.y);
    Serial.print(", ");
    Serial.print(pose.theta);
    Serial.print(",           ");
    Serial.print(target_x);
    Serial.print(",");
    Serial.println(target_y);
  }
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

void checkEnableDemand() {
  // Wait for demand to be enabled after initialisation delay
  if (!enable_demand && millis() > enable_demand_ts + enable_demand_ms) {
    enable_demand = true;
    left_pid.reset();
    right_pid.reset();
    heading.reset();
  }
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
  checkMoving();
  computeSpeed(); // update global variables with new speed estimates
  if (enable_demand) {
    obeyDemand(); // Set motor PWM according to demand
  }
  updatePose(); // update kinematics
  driftWaypoints();
  calcCalibratedMag(max_mag_readings, min_mag_readings);
  checkState();
}
