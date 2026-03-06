// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

// These two commands mean that this header file
// will attempt to use some global variables of
// the same name found in another header file.
// From encoders.h
extern volatile long count_e0;
extern volatile long count_e1;

// Some global definitions concerning
// the robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Labsheet 4)
const float count_per_rev = 358.3;   // From documentation - correct.
const float wheel_radius  = 16.8;    // mm, could vary - calibrate. 
const float wheel_sep     = 44.0;    // mm, from centre of robot to wheel centre 
                                     //     - could vary, calibrate

// Take the circumference of the wheel and divide by the 
// number of counts per revolution. This provides the mm
// travelled per encoder count.
const float mm_per_count  = ( 2.0 * wheel_radius * PI ) / count_per_rev;

// Class to track robot position.
class Kinematics_c {
  private:
    // Pose
    float x, y, theta;
    long last_e1;
    long last_e0;
    
    unsigned long pose_update_ts;
    unsigned long pose_update_ms;

    float desiredAngle(float target_x, float target_y) {
      return atan2(target_y-y, target_x-x);
    }

    void update() {
      long delta_e1;  // change in counts
      long delta_e0;  // change in counts
      float mean_delta;
       
      float x_contribution;   // linear translation
      float th_contribution;  // rotation

      // How many counts since last update()?
      delta_e1 = count_e1 - last_e1;
      delta_e0 = count_e0 - last_e0;

      // Used last encoder values, so now update to
      // current for next iteration
      last_e1 = count_e1;
      last_e0 = count_e0;
        
      // Work out x contribution in local frame.
      mean_delta = (float)delta_e1;
      mean_delta += (float)delta_e0;
      mean_delta /= 2.0;

      x_contribution = mean_delta * mm_per_count;

      // Work out rotation in local frame
      th_contribution = (float)delta_e0;
      th_contribution -= (float)delta_e1;
      th_contribution *= mm_per_count;
      th_contribution /= (wheel_sep *2.0);

      // Update global frame by taking these
      // local contributions, projecting a point
      // and adding to global pose.
      x = x + x_contribution * cos( theta );
      y = y + x_contribution * sin( theta );
      theta = theta + th_contribution;
      // Done!
    }

  public:
    // Constructor, must exist.
    Kinematics_c() {} 

    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      last_e0 = count_e0; // Initisalise last count to current count
      last_e1 = count_e1; // Initisalise last count to current count
      x = start_x;
      y = start_y;
      theta = start_th;

      pose_update_ts = millis();
      pose_update_ms = 30;
    }

    float distSq(float target_x, float target_y) {
      return pow(x - target_x, 2) + pow(y - target_y, 2);
    }

    float dist(float target_x, float target_y) {
      return sqrt(distSq(target_x, target_y));
    }

    float smallestAngle(float desired_angle) {
      float diff = desired_angle - theta;
      while (diff > PI) diff -= 2.0f * PI;
      while (diff < -PI) diff += 2.0f * PI;
      return diff;
    }

    float angleDiff(float target_x, float target_y) {
      return smallestAngle(desiredAngle(target_x, target_y));
    }

    float angleToOrigin() {
      return atan2(-x, -y);
    }

    void checkUpdate() {
      unsigned long now = millis();
      if (now <= pose_update_ts + pose_update_ms) {return;}
      pose_update_ts = now;
      update();
    }
};


#endif
