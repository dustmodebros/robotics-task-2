#ifndef TUNING_H
#define TUNING_H

const float count_per_rev = 358.3;   // From documentation - correct.
const float wheel_radius  = 17.19;    // mm, could vary - calibrate. 
const float wheel_sep     = 45.14;    // mm, from centre of robot to wheel centre 
																		 //     - could vary, calibrate

const float left_p = 0.27;
const float left_i = 0.0011;
const float left_d = 0.001;
const float right_p = 0.31;
const float right_i = 0.001;
const float right_d = 0.01;
const float heading_p = 0.7;
const float heading_i = 0.0;
const float heading_d = 0.0;
#endif
