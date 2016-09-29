#include "Motor.h"

Motor::Motor (int speedpin,  int inApin, int inBpin) {
  this->speedpin   = speedpin ;
  this->inApin  = inApin;
  this->inBpin  = inBpin;

}

void Motor::initial() {
  pinMode(speedpin, OUTPUT);
  pinMode(inApin, OUTPUT);
  pinMode(inBpin, OUTPUT);
  analogWrite(speedpin, 0);
  digitalWrite(inApin, LOW);
  digitalWrite(inBpin, HIGH);
}

void Motor::setVel(int velocity) {
  analogWrite(speedpin, velocity);
}

void Motor::updat(int sp) {
  bool back = false;
  if (sp < 0) {
    back = true;
    sp = -sp;
  }
  if (sp > maxPWM) sp = maxPWM;
  if (sp < minPWM) sp = 0;

  if (back) {
      setDir(1, 0);
    } else {
      setDir(0, 1);
    }
  setVel(sp);
}

void Motor::setDir(bool inA, bool inB){
  digitalWrite(inApin, inA);
  digitalWrite(inBpin, inB);
}


