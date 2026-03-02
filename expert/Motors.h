
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
#define L_PWM 10 // This is correct.
#define L_DIR 16 // This is correct.
#define R_PWM 9 // This is correct.
#define R_DIR 15 // This is correct.

// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MAX_PWM 180

// Class to operate the motors.
class Motors_c {

  public:

    // Constructor, must exist.
    Motors_c() {
      // Leave empty. Ensure initialise() is called
      // instead.
    }

    // Use this function to initialise the pins that
    // will control the motors, and decide what first
    // value they should have.
    void initialise() {

      pinMode( L_PWM , OUTPUT );
      pinMode( L_DIR , OUTPUT );
      pinMode( R_PWM , OUTPUT );
      pinMode( R_DIR , OUTPUT );

      digitalWrite( L_DIR, HIGH );
      digitalWrite( R_DIR, HIGH );

      analogWrite( L_PWM , 0 );
      analogWrite( R_PWM , 0 );

    } // End of initialise()


    // This function will be used to send a power value
    // to the motors.
    //
    // The power sent to the motors is created by the
    // analogWrite() function, which is producing PWM.
    // analogWrite() is intended to use a range between
    // [0:255].
    //
    // This function takes two input arguments: "left_pwr"
    // and "right_pwr", (pwr = power) and they are of the
    // type float. A float might be a value like 0.01, or
    // -150.6
    void setPWM( float left_pwr, float right_pwr ) {
      // max: 1, min: -1
      // 1 becomes 255 and positive, -1 becomes 255 and negative
      // so get abs of float, and multiply by 255
//      Serial.print("Setting motor power to - Left power: ");
//      Serial.print(abs(left_pwr) * 255);
//      Serial.print(", Right power: ");
//      Serial.println(abs(right_pwr) * 255);

      if (abs(left_pwr) > 1 or abs(right_pwr) > 1){
        analogWrite( L_PWM , 0);
        analogWrite( R_PWM , 0);

        return;
      }

      
      
      analogWrite( L_PWM , abs(left_pwr) * MAX_PWM);
      analogWrite( R_PWM , abs(right_pwr) * MAX_PWM);

      // What should happen if the request for left_pwr
      // is less than 0? Recall, how are these motors
      // operated in terms of the pins used?
      if ( left_pwr <= 0 ) {
        digitalWrite( L_DIR, HIGH );
      } else {
        digitalWrite( L_DIR, LOW );
      }

      // What should happen if the request for right_pwr
      // is less than 0? Recall, how are these motors
      // operated in terms of the pins used?
      if ( right_pwr <= 0 ) {
        digitalWrite( R_DIR, HIGH );
      } else {
        digitalWrite( R_DIR, LOW );
      }
      
      return;

    } // End of setPWM()


}; // End of Motors_c class definition.



#endif
