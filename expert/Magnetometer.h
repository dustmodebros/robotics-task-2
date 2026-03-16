/***************************************
 ,        .       .           .     ,-.  
 |        |       |           |        ) 
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /  
 |    | | | | `-. | | |-' |-' |      /   
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--' 
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {
  private:
    LIS3MDL mag;
    float maximum[MAX_AXIS] = {-9999.0, -9999.0, -9999.0};
    float minimum[MAX_AXIS] = {9999.0, 9999.0, 9999.0};
    float readings[MAX_AXIS];
    unsigned long mag_ts;

    // returns affirmative if new measurements were taken
    bool getReadings() {
      // ensure the magnetometer is not engaged too frequently
      if (millis() < mag_ts + 100) {return false;}
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
      mag_ts = millis();
      return true;
    } // End of getReadings()

  public:
    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {
      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if ( !mag.init() ) {
        return false;
      } else {
        return true;
      }
      mag.enableDefault();
    }

    void calibrate() {
      if (!getReadings()) {return;}
      for (int i = 0; i < MAX_AXIS; i++) {
        maximum[i] = max(readings[i], maximum[i]);
        minimum[i] = min(readings[i], minimum[i]);
      }
    }

    void doCalibratedReadings() {
      if (!getReadings()) {return;}
      for (int i = 0; i < MAX_AXIS; i++) {
        readings[i] = 2.0f * (readings[i] - minimum[i]) / (maximum[i] - minimum[i]) - 1.0f;
      }
    }

    float convertToMagnitude() {
      float sum = 0, reading;
      for (int i = 0; i < MAX_AXIS; i++) {
        reading = readings[i];
        sum += reading * reading;
      }
      return sqrt(sum);
}

}; // End of Magnetometer_c class definition

#endif
