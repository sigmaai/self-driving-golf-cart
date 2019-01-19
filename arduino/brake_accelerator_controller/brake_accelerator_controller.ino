//
// Arduino code for brake & accelerator control
// Cruise control module | self-driving golf cart project
// (c) Yongyang Nie, All Rights Reserved
//

#include <SPI.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define RPWM 7
#define LPWM 6
#define M_PI 3.14159265359
#define POT_MAX 100
#define POT_MIN 0
#define LA_MIN 0
#define LA_MAX 0
#define LA_PIN 0

ros::NodeHandle nh;

boolean joystick_enabled = false;
float target_speed = 0.0;
float cmd_val = 0.0;
long actuator_pos = 0;
const int slave_Select_Pin = 10;  // set pin 10 as the slave select for the digital pot:

void cruise_callback( const std_msgs::Float32& cmd_msg) {


}

void joystick_callback( const std_msgs::Float32& cmd_msg) {

  if (joystick_enabled) {

    if (cmd_val > 0) {
      cmd_val = mapf(cmd_msg.data, 0, 1, POT_MIN, POT_MAX);
    } else if (cmd_val < 0) {
      // engage brakes
    }
  }
}

void joystick_enabled_callback( const std_msgs::Bool& cmd_msg) {
  
  joystick_enabled = cmd_msg.data;
}

// ----------------------------------------------------------------------------------------
// declare all subscribers
ros::Subscriber<std_msgs::Float32> sub1("/vehicle/dbw/cruise_cmds/", cruise_callback);
ros::Subscriber<std_msgs::Float32> sub2("/sensor/joystick/right_stick_y", joystick_callback);
ros::Subscriber<std_msgs::Bool> sub3("/sensor/joystick/enabled", joystick_enabled_callback);

// declare the publisher
std_msgs::Float32 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/brake/actuator_position", &pos_msg);
// ----------------------------------------------------------------------------------------

void setup() {

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  nh.advertise(pos_pub);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  actuator_pos = analogRead(LA_PIN);

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);

  // initialize SPI:
  SPI.begin();
}

void loop() {

  potWrite(slave_Select_Pin, B00010001, cmd_val);
  potWrite(slave_Select_Pin, B00010010, cmd_val);

  /*
    if (cruise_speed == -1) {
      potWrite(slave_Select_Pin, B00010001, 0);
      potWrite(slave_Select_Pin, B00010010, 0);

      press_break(1);
      delay(1000);
      release_break(1);

      Serial.println("brakes engaged");
    }*/
}


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
