
// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define NUM_SENSORS 5

// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11,
// sensors_pins[1] is A0.
const int sensor_pins[ NUM_SENSORS ] = { A11, A0, A2, A3, A4 };

// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN   11


// Class to operate the linesensors.
class LineSensors_c {
  private:
    float readings[ NUM_SENSORS ];
    float minimum[ NUM_SENSORS ] = {1024.0, 1024.0, 1024.0, 1024.0, 1024.0};
    float maximum[ NUM_SENSORS ] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float scaling[ NUM_SENSORS ];

    float calibrated[ NUM_SENSORS ];
    bool onLine[ NUM_SENSORS];

    void readSensorsADC() {
      for(int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        readings[sensor] = analogRead(sensor_pins[sensor]);
      }
    }    

    void calcCalibratedADC() {
      // Get latest readings (raw values)
      readSensorsADC();
      float min;
      // Apply calibration values, store in calibrated[]
      for(int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        min = minimum[sensor];
        calibrated[sensor] = (readings[sensor] - min) / (maximum[sensor]- min);
      }
      
    } // End of calcCalibratedADC()

  public:
    // Constructor, must exist.
    LineSensors_c() {
      // leave this empty
    }

    void initialiseForADC() {

      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

      // Configure the line sensor pins
      // DN1, DN2, DN3, DN4, DN5.
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
        pinMode( sensor_pins[sensor], INPUT_PULLUP );
      }
    }

    void updateOnLine(float min_values[NUM_SENSORS], float max_values[NUM_SENSORS]){
      calcCalibratedADC();
      for (int i=0; i<NUM_SENSORS; i++){
        onLine[i] = calibrated[i] < 0.5;
      }
    }

    void calibrate() {
      readSensorsADC();
      for (int i = 0; i < 5; i++) {
        maximum[i] = max(readings[i], maximum[i]);
        minimum[i] = min(readings[i], minimum[i]);
      }
    }

}; // End of LineSensor_c class defintion

#endif
