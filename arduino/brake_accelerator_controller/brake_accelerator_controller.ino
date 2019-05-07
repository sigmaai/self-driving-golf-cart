//
// Arduino code for brake & accelerator control
// Cruise control module | self-driving golf cart project
// (c) Yongyang Nie, All Rights Reserved
//
// 1 is in
// 0 is out

#include <SPI.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#define RPWM 7
#define LPWM 6
#define POT_MAX 150
#define POT_MIN 30
#define LA_MIN 250.0
#define LA_MAX 450.0 // 380
#define LA_PIN 0

ros::NodeHandle nh;

boolean joystick_enabled = false;
float cmd_val = 0.0;
int actuator_pos = 0;
float actuator_target_pos = 0;
const int slave_Select_Pin = 10;  // set pin 10 as the slave select for the digital pot:
long lastest_msg_time = 0;
float current_vel = 0.0;
float desired_vel = 0.0;

void joystick_callback( const std_msgs::Float32& cmd_msg) {

  if (joystick_enabled == 1) {

    if (cmd_msg.data >= 0) {

      // make sure the brake is released
      actuator_target_pos = LA_MAX;
      cmd_val = mapf(cmd_msg.data, 0, 1, POT_MIN, POT_MAX);

    } else {

      // engage brakes
      cmd_val = POT_MIN;
      float inverted_input = 1.0 + cmd_msg.data;
      actuator_target_pos = mapf(inverted_input, 0, 1, LA_MIN, LA_MAX);

    }
  }
}

void joystick_enabled_callback( const std_msgs::Bool& cmd_msg) {

  joystick_enabled = cmd_msg.data;
}

void header_callback( const std_msgs::Header& header_msg) {
  lastest_msg_time = header_msg.stamp.sec;
}

void vel_callback(const std_msgs::Float32MultiArray& vel_msg) {
  current_vel = vel_msg.data[0];
  desired_vel = vel_msg.data[1];
}

// ----------------------------------------------------------------------------------------
// declare all subscribers
ros::Subscriber<std_msgs::Header> header_sub("/vehicle/dbw/vel_cmd_header", header_callback);
ros::Subscriber<std_msgs::Float32> vel_sub("/vehicle/dbw/velocity", vel_callback);
ros::Subscriber<std_msgs::Float32> sub2("/sensor/joystick/right_stick_y", joystick_callback);
ros::Subscriber<std_msgs::Bool> sub3("/sensor/joystick/enabled", joystick_enabled_callback);

// declare the publisher
std_msgs::Float32 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/brake/actuator_position", &pos_msg);
// ----------------------------------------------------------------------------------------

void setup() {

  nh.initNode();
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(header_sub);
  nh.subscribe(vel_sub);

  nh.advertise(pos_pub);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  actuator_pos = analogRead(LA_PIN);
  actuator_target_pos = actuator_pos;

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);

  // initialize SPI:
  SPI.begin();
}

void loop() {

  // accelerator control
  potWrite(slave_Select_Pin, B00010001, cmd_val);
  potWrite(slave_Select_Pin, B00010010, cmd_val);

  // linear actuator brake control
  if (abs(actuator_pos - actuator_target_pos) > 10) {
    if (actuator_pos < actuator_target_pos)
      move_actuator(255, 0);
    else if (actuator_pos > actuator_target_pos)
      move_actuator(255, 1);
  }
  else {
    stop_actuator();
  }

  // publish linear actuator current position
  actuator_pos = analogRead(LA_PIN);
  pos_msg.data = actuator_pos;
  pos_pub.publish(&pos_msg);

  nh.spinOnce();
  delay(5);
}

//
// ================== Helper methods ================== 
// 

void potWrite(int slaveSelectPin, byte address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);
}

void press_break(float amount) {


}

void release_break(float amount) {


}

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

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
