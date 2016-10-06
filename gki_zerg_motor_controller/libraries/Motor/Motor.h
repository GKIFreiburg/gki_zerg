#include <Arduino.h>
#ifndef __MOTOR__
#define __MOTOR__
class Motor {
  public:
    // Constants
    const int maxPWM = 255;
    const int minPWM = 8;
    //Pins that are controlled
    int speedpin;
    int inApin;
    int inBpin;
    
    //Constructor maps the pins
    Motor(int speedpin, int inApin, int inBpin);
    //Initializes the Motors and sets velocity(must be between 0 and 255);
    void initial();
    // Sets PWM Signal
    void setVel(int velocity);
    // Sets the Directions
    void setDir(bool inA, bool inB);
    // Controls the driver
    void updat(int sp);
};
#endif
