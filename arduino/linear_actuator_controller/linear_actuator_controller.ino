//
// motor controller code by Neil Nie & Michael Meng
// Steering Controller Module | self driving golf cart
// (c) Yongyang Nie, All rights reserved
//

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define RPWM 7
#define LPWM 6
#define M_PI 3.14159265359
#define THRESHOLD 0.5

#define la_max 700
#define la_min 195 
#define pot_pin 0

unsigned long count;      // count for encode *might have duplicate variables
double pos = 0.0;         // steering position
boolean joystick_enabled = true;
float cmd_val = 0.0;

ros::NodeHandle nh;

// the ros callback methods must be declared before the ros subscriber statement.

void steering_callback( const std_msgs::Float32& cmd_msg) {

  if (!joystick_enabled) {
    // float cmd = map(cmd_msg.data, -1, 1, la_min, la_max);
    // steering(cmd);
  }
}

void joystick_callback( const std_msgs::Float32& cmd_msg) {

  if (joystick_enabled) {
    cmd_val = cmd_msg.data;
    steering(cmd_val);
  }
}

void joystick_enabled_callback( const std_msgs::Bool& cmd_msg) {
  joystick_enabled = cmd_msg.data;
}

ros::Subscriber<std_msgs::Float32> sub1("/vehicle/dbw/steering_cmds/", steering_callback);
ros::Subscriber<std_msgs::Float32> sub2("/sensor/joystick/left_stick_x", joystick_callback);
ros::Subscriber<std_msgs::Bool> sub3("/sensor/joystick/enabled", joystick_enabled_callback);

std_msgs::Float32 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/steering/actuator_position", &pos_msg);

void setup() {

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  nh.advertise(pos_pub);
  
  //setup motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pos = 0.0;
}

void loop() {
  
  pos = analogRead(pot_pin);
  
  pos_msg.data = pos;
  pos_pub.publish(&pos_msg);
  nh.spinOnce();
  delay(5);
}

// This method MAY BE WRONG
void steering(double input) {

  if (input > 0 && pos > la_min){
    move_actuator(255, 0);
  }else if (input < 0 && pos < la_max){
    move_actuator(255, 1);
  }else{
    stop_actuator();
  }
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

