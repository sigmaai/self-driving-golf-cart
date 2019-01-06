//
// motor controller code by Neil Nie & Michael Meng
// Steering Controller Module | self driving golf cart
// (c) Yongyang Nie, All rights reserved
//


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#define RPWM 7
#define LPWM 6
#define M_PI 3.14159265359
#define THRESHOLD 0.5

#define la_max 1000
#define la_min 10
#define encoder_a 2
#define encoder_b 3

unsigned long count;      // count for encode *might have duplicate variables
double pos = 0.0;         // steering position
float steering_value;     // steering value
boolean joystick_enabled;

ros::NodeHandle nh;

// the ros callback methods must be declared before the ros subscriber statement.

void steering_callback( const std_msgs::Float32& cmd_msg){

  // int buttons[10] = cmd_msg.buttons;
  float cmd_val = map(cmd_msg.data, -1, 1, la_min, la_max);
  steering(cmd_val);
}

void joystick_callback( const sensor_msgs::Joy& cmd_msg) {
  float *axes = cmd_msg.axes;
  // int buttons[10] = cmd_msg.buttons;
  float cmd_val = map(axes[0], -1, 1, la_min, la_max);
  steering(cmd_val);
}

void joystick_enabled_callback( const std_msgs::Bool& cmd_msg){  
  joystick_enabled = cmd_msg.data;
}

ros::Subscriber<std_msgs::Float32> sub1("/vehicle/dbw/steering_cmds/", steering_callback);
ros::Subscriber<sensor_msgs::Joy> sub2("/sensor/joystick/joy", joystick_callback);
ros::Subscriber<std_msgs::Bool> sub3("/sensor/joystick/enabled", joystick_enabled_callback);

void setup() {

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  //setup encoder
  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);
  
  //setup motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  
  pos = 0.0;
}

void loop() {
  nh.spinOnce();
  delay(1);
}

// This method MAY BE WRONG
void steering(double angle) {

  while (1) {
    if (angle > 0) {
      if (count > angle) {
        stop_actuator();
        break;
      }
      else
        move_actuator(255, 0);

    } else {
      if (count < angle) {
        stop_actuator();
        break;
      }
      else
        move_actuator(255, 1);
    }
  }
  pos = count;
}

// BEGIN ---- Linear Actuator Encoder Interrupt ----------------------
void encoderPinChangeA() {
  count += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  count += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}

// END ------ Linear Actuator Encoder Interrupt ----------------------

// BEGIN ---- Helper Methods  ----------------------------------------

void move_actuator(int spd, boolean dir) {
  if (dir) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, spd);
  } else {
    analogWrite(LPWM, spd);
    analogWrite(RPWM, 0);
  }
}

void stop_actuator() {
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(10);
}

// END ------ Helper Methods  ----------------------------------------

