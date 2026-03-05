
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
  
  public:

    // Store your readings into this array.
    // You can then access these readings elsewhere
    // by using the syntax line_sensors.readings[n];
    // Where n is a value [0:4]
    float readings[ NUM_SENSORS ];

    // Variables to store calibration constants. 
    // Make use of these as a part of the exercises 
    // in labsheet 2.
    float minimum[ NUM_SENSORS ] = {1024.0, 1024.0, 1024.0, 1024.0, 1024.0};
    float maximum[ NUM_SENSORS ] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float scaling[ NUM_SENSORS ];

    // Variable to store the calculated calibrated
    // (corrected) readings. Needs to be updated via
    // a function call, which is completed in 
    // labsheet 2.
    float calibrated[ NUM_SENSORS ];

    bool onLine[ NUM_SENSORS];

    // Constructor, must exist.
    LineSensors_c() {
      // leave this empty
    }

    // Refer to Labsheet 2: Approach 1
    // Fix areas marked ????
    // Use this function to setup the pins required
    // to perform a read of the line sensors using
    // the ADC.
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
      
    } // End of initialiseForADC()



    // Refer to Labsheet 2: Approach 1
    // Fix areas marked ????
    // This function is as simple as using a call to 
    // analogRead()
    void readSensorsADC() {

      // First, initialise the pins.
      // You need to complete this function (above).
      initialiseForADC();

      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
               readings[sensor] = analogRead(sensor_pins[sensor]);
      }

    } // End of readSensorsADC()

    

    // Use this function to apply the calibration values
    // that were captured in your calibration routine.
    // Therefore, you will need to write a calibration
    // routine (see Labsheet 2)
    void calcCalibratedADC( float min_values[NUM_SENSORS], float max_values[NUM_SENSORS]) {

      // Get latest readings (raw values)
      readSensorsADC();

      // Apply calibration values, store in calibrated[]
      for( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
              calibrated[sensor] = (readings[sensor] - min_values[sensor]) / (max_values[sensor]- min_values[sensor]);
      }
      
    } // End of calcCalibratedADC()



    void updateOnLine(float min_values[NUM_SENSORS], float max_values[NUM_SENSORS]){
      calcCalibratedADC( min_values, max_values);
      for (int i=0; i<NUM_SENSORS; i++){
        onLine[i] = calibrated[i] < 0.5;
      }
    }

  
    // Part of the Advanced Exercises for Labsheet 2
    void initialiseForDigital() {
      
      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

    } // End of initialiseForDigital()

    // Part of the Advanced Exercises for Labsheet 2
    void readSensorsDigital() {
    //  ???
    } // End of readSensorsDigital()

    void calibrate() {
      readSensorsADC();
      for (int i = 0; i < 5; i++) {
        maximum[i] = max(line_sensors.readings[i], maximum[i]);
        minimum[i] = min(line_sensors.readings[i], minimum[i]);
      }
      //unfinished IR sensor implementation.
    }

}; // End of LineSensor_c class defintion



#endif
