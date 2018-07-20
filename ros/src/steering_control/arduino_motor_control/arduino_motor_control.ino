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

#define RPWM 7
#define LPWM 6
#define LEN 6 //length of the actual message
// #define FPS 1
#define M_PI 3.14159265359
#define THRESHOLD 0.5

unsigned long count; //count for encode
double pos = 0.0; //steering position
float steering_value; //steering value
boolean dir; //steering direction

//function when receive encoder interrupt
const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital

ros::NodeHandle nh;

void steering_callback( const std_msgs::Float32& cmd_msg) {
  float angle = cmd_msg.data;
  steering(angle);
}

ros::Subscriber<std_msgs::Float32> sub("/vehicle/dbw/steering_cmds/", steering_callback);

void setup() {

  nh.initNode();
  nh.subscribe(sub);

  //setup encoder
  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);
  //setup motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pos = 0.0;
  dir = 0;
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void steering(double angle) {

  while (1) {
    if (angle > 0) {
      if (getRadian(count) > angle) {
        st();
        break;
      }
      else
        mv(255, 0);

    } else {
      if (getRadian(count) < angle) {
        st();
        break;
      }
      else
        mv(255, 1);
    }
  }

  pos = getRadian(count);
}

void encoderPinChangeA() {
  count += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  count += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}

//turn encoder count to radian value
double getRadian(int c) {
  return (double)c * 0.6428571429 * (M_PI / 180.0);
}

void mv(int spd, boolean dir) {
  if (dir) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, spd);
  } else {
    analogWrite(LPWM, spd);
    analogWrite(RPWM, 0);
  }
}

void st() {
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(10);
}

