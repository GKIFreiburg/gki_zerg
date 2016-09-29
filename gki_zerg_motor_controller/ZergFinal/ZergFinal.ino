#include <ros.h>
#include <ros/time.h>
#include <Arduino.h>
#include <Motor.h>
#include <OptEnc.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>


#define LEFT 0
#define RIGHT 1

OptEnc optenc;
Motor motL(6, 52, 50);
Motor motR(7, 51, 53);


// Constants
// Robot
const float wheeldiameter = 0.12; //diameter
const float perimeter = wheeldiameter * 3.14; //perimaeter of a wheel in m
const float robotWidth = 0.4;
// Encoder
const long countsPerRot = 7880; //counts per rotation
const float dist_pulse = perimeter / countsPerRot; //distance driven in one pulse
//SpeedRanges
const float maxSpeed = 2.0;
const float maxRad = 3.0;
//PID Controller
const long pidTu = 600;
const float pidKP = 80;
const float pidKI = 0.8 * pidKP / pidTu;
const float pidKD = 0 * pidKP * pidTu / 8;
//Pins
const int stopPin = 49;

//Program
//Cycletime of one loop
int cycletime = 10;
//Flags
bool stopJoy;
bool stopFlag;

//Ticks of the encoders
long odo[4];
//Speed Value sent to the controller
float input_speed[2];

//internal PID variables
long pos_old[4];
float speed_old[4];
float errorI[2];
float oldErrorD[2];

//meassured speed each for one side
float speed_measure[2];

//pwm signal sent to motor
int pwm_speed[2];

struct pid {
  float kp;
  float ki;
  float kd;
};

//speed of each tyre
float mov[4];
char t_loop;

//timer for safety button
long timeout;

//Ros stuff
ros::NodeHandle nh;
//publisher
std_msgs::UInt8 progTime_msg;
std_msgs::Bool stop_msg;
geometry_msgs::TwistStamped speed_msg;
ros::Publisher chatter("debug/prog_time", &progTime_msg);
ros::Publisher chat2("sensors/wheelspeed", &speed_msg);
ros::Publisher stopper("event/estop", &stop_msg);
//subscriber
void cbf(const geometry_msgs::Twist& msg)
{
  if (stopFlag) {
    input_speed[LEFT] = msg.linear.x * maxSpeed - (maxRad * 0.5 * msg.angular.z) * robotWidth;
    input_speed[RIGHT] = msg.linear.x * maxSpeed + (maxRad * 0.5 * msg.angular.z) * robotWidth;
    if (input_speed[LEFT] > maxSpeed) {
      input_speed[RIGHT] -= input_speed[LEFT] - maxSpeed;
      input_speed[LEFT] = maxSpeed;
    }
    if (input_speed[RIGHT] > maxSpeed) {
      input_speed[LEFT] -= input_speed[RIGHT] - maxSpeed;
      input_speed[RIGHT] = maxSpeed;
    }
    if (input_speed[LEFT] < -1 * maxSpeed) {
      input_speed[RIGHT] -= input_speed[LEFT] + maxSpeed;
      input_speed[LEFT] = -1 * maxSpeed;
    }
    if (input_speed[RIGHT] < -1 * maxSpeed) {
      input_speed[LEFT] -= input_speed[RIGHT] + maxSpeed;
      input_speed[RIGHT] = -1 * maxSpeed;
    }
  } else {
    input_speed[RIGHT] = 0;
    input_speed[LEFT] = 0;
  }
  timeout = millis();
}

void joyStopIn(const std_msgs::Bool& msg)
{
  stopJoy=msg.data;
}

ros::Subscriber<geometry_msgs::Twist> sub("commands/velocity", cbf);
ros::Subscriber<std_msgs::Bool> joysub ("commands/estop", joyStopIn);

void setup() {
  const int freqOutputPin = 10;
  pinMode(freqOutputPin, OUTPUT);
  Serial.begin(9600);
  optenc.setupfrq();

  pinMode(stopPin, INPUT);

  optenc.mapPins(61, 60, 59, 58, 56);
  optenc.mapData(63, 69, 68, 67, 64, 65, 66, 62);
  for (int i = 62; i < 70; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 56; i < 62; i++) {
    pinMode(i, OUTPUT);
  }
  digitalWrite(optenc.sel1pin, HIGH);
  digitalWrite(optenc.sel2pin, LOW);
  digitalWrite(optenc.resetpin, HIGH);
  digitalWrite(optenc.oepin, 1);
  digitalWrite(optenc.selxypin, HIGH);

  motR.initial();
  motR.setVel(0);
  motR.setDir(0, 1);
  motL.initial();
  motL.setVel(0);
  motL.setDir(0, 1);

  errorI[LEFT] = 0;
  errorI[RIGHT] = 0;

  timeout = millis();
  stopJoy = true;
  stopFlag = true;
  
  //ros stuff
  speed_msg.header.frame_id = "base_link";
  
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(chat2);
  nh.advertise(stopper);
  nh.subscribe(sub);
  nh.subscribe(joysub);
}

void loop() {
  if (digitalRead(stopPin)&&(stopJoy)) {
    stop_msg.data = true;
    stopFlag = true;
  } else {
    stop_msg.data = false;
    stopFlag = false;
  }
  unsigned long t_start = millis();
  if (timeout + 500 < t_start) {
    input_speed[RIGHT] = 0;
    input_speed[LEFT] = 0;
  }
  optenc.getOdo();
  measureSpeed();
  for (int i = 0; i < 2; i++) {
    float errorterm  = input_speed[i] - speed_measure[i];
    pid pidR;
    pidR.kp = (getP(errorterm , pidKP));
    pidR.ki = (getI(i, errorterm , pidKI));
    pidR.kd = (getD(i, errorterm , pidKD));
    pwm_speed[i] = pidR.kd + pidR.ki + pidR.kp;
    if (input_speed[i] == 0) {
      pwm_speed[i] = 0;
      errorI[i] = 0;
      if (speed_measure[i] == 0) {
        //      } else {
        //        if (i) {
        //          motR.setDir(0, 0);
        //          pwm_speed[i]=40;
        //        } else {
        //          motL.setDir(0, 0);
        //          pwm_speed[i]=40;
        //        }
      }
    }
  }
  motR.updat(pwm_speed[RIGHT]);
  motL.updat(pwm_speed[LEFT]);
  speed_msg.twist.linear.x = (speed_measure[RIGHT] + speed_measure[LEFT]) / 2;
  speed_msg.twist.angular.z = (speed_measure[RIGHT] - speed_measure[LEFT]) / robotWidth;
  speed_msg.header.stamp = nh.now();
  chat2.publish( &speed_msg );
  stopper.publish( &stop_msg );
  progTime_msg.data = t_loop;
  chatter.publish( &progTime_msg );
  nh.spinOnce();
  t_loop = millis() - t_start;
  if (t_loop < cycletime) delay (cycletime - t_loop);
}

float updateSpeed(int index, long pos) {
  float dist_drive = dist_pulse * (pos - pos_old[index]);
  pos_old[index] = pos;
  float out = dist_drive / cycletime * 1000;
  speed_old[index] = out;
  return out;
}

float getP(float error, float kp) {
  return error * kp;
}

float getI(bool right, float error, float ki) {
  errorI[right] += error * cycletime * ki;
  return errorI[right];
}

float getD(bool right, float error, float kd) {
  float dError =  error - oldErrorD[right];
  oldErrorD[right] = error;
  return kd * dError / cycletime;
}

void measureSpeed() {
  mov[0] = updateSpeed(0, optenc.odo.hr);
  mov[1] = updateSpeed(1, optenc.odo.vr);
  speed_measure[RIGHT] = mov[0];
  if (abs(mov[1]) < abs(mov[0])) speed_measure[RIGHT] = mov[1];

  mov[2] = updateSpeed(2, optenc.odo.hl);
  mov[3] = updateSpeed(3, optenc.odo.vl);
  speed_measure[LEFT] = mov[2];
  if (abs(mov[3]) < abs(mov[2])) speed_measure[LEFT] = mov[3];

}
