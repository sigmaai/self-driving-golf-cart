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
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#define RPWM 7
#define LPWM 6
#define POT_MAX 120
#define POT_MID 50
#define POT_MIN 25
#define LA_MIN 15.0 // correct 260
#define LA_MAX 155.0 // correct 380
#define LA_PIN 0

ros::NodeHandle nh;

int actuator_pos = 0;
float actuator_target_pos = 0;
const int slave_Select_Pin = 10;  // set pin 10 as the slave select for the digital pot:

// make sure that the ROS messages are synced
// and there is no delay in the path planner.
long current_vel_time = 0;
long desired_vel_time = 0;

float cmd_val = 0.0;
float current_velocity = 0.0;
float pot_input = 0.0;
float remote_accel_val = 0.0;

boolean killed = false;
boolean velocity_callback = false;
boolean joystick_enabled = false;
boolean go = false;
boolean rc_control = false;

int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;
int ch4;
int ch5;
int log_val;

void joystick_enabled_callback( const std_msgs::Bool& cmd_msg) {
  joystick_enabled = cmd_msg.data;
}

//void time_sync_callback( const std_msgs::Header& header_msg) {
//  current_vel_time = header_msg.stamp.nsec;
//}

void set_go_status(const std_msgs::Bool &cmd_msg) {
  
}

void current_vel_callback(const std_msgs::Float32& cmd_msg) {
  current_velocity = cmd_msg.data;
}

// ===== driver control ======
void joystick_callback( const std_msgs::Float32& cmd_msg) {

  if (joystick_enabled && !killed) {

    if (cmd_msg.data >= 0) {

      // make sure the brake is released
      actuator_target_pos = LA_MAX;
      cmd_val = mapf(cmd_msg.data, 0, 1, POT_MIN, POT_MID);

    } else {

      // engage brakes
      cmd_val = POT_MIN;
      float inverted_input = 1.0 + cmd_msg.data;
      actuator_target_pos = mapf(inverted_input, 0, 1, LA_MIN, LA_MAX);
    }
  }
}

// ========================================================
// ===================== R C Control ======================

//void dir_accel_callback(const std_msgs::Float32& cmd_msg) {
//
//  // make sure the brake is released
//  if (!killed) {
//    if (cmd_msg.data < 0) {
//      remote_accel_val = 0;
//      float inverted_input = 1.0 + cmd_msg.data;
//      actuator_target_pos = mapf(inverted_input, 0, 1, LA_MIN, LA_MAX);
//    } else
//      remote_accel_val = mapf(cmd_msg.data, 0, 1, POT_MIN, POT_MID);
//  }
//}

// =========================================================
// =========================================================
void desired_vel_callback(const std_msgs::Float32& cmd_msg) {

  if (!joystick_enabled && !killed) {
    float desired_velocity = cmd_msg.data;
    float delta_vel = desired_velocity - current_velocity;

    // vehicle needs to accelerate
    if (delta_vel > 0.0 && desired_velocity >= 0.08) {

      log_val = -1;

      actuator_target_pos = LA_MAX;
      pot_input = (pot_input < 1.0) ? pot_input + 0.01 : 1.0;
      remote_accel_val = mapf(pot_input, 0, 1, POT_MIN, POT_MID);

    } else if (delta_vel < 0.0 || desired_velocity < 0.8) {

      remote_accel_val = POT_MIN;

      // vehicle needs to decelerate
      if (delta_vel < -0.08 && delta_vel > -0.12) {
        log_val = 1;
        actuator_target_pos = LA_MIN + (LA_MAX - LA_MIN) * 0.45;
      } else if (delta_vel <= -0.12 && delta_vel >= -0.25) {
        log_val = 2;
        actuator_target_pos = LA_MIN + (LA_MAX - LA_MIN) * 0.33;
        // if dramatic deceleration or low speed
      } else {
        log_val = 3;
        actuator_target_pos = LA_MIN;
      }
    }

  }
  // TODO: This needs to be fixed.
  // two times, current velocity timestamp
  //            desired velocity timestamp
  // if the two has a greater than 0.5 second difference. slowly stop the vehicle.
  // else
  // actuator_target_pos = LA_MIN + (LA_MAX - LA_MIN) * 0.85;
}

// ----------------------------------------------------------------------------------------
// declare all subscribers
// ----------------------------------------------------------------------------------------

std_msgs::Int16 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/brake/actuator_position", &pos_msg);

std_msgs::Bool kill_msg;
ros::Publisher kill_pub("/vehicle/safety/killed", &kill_msg);

void setup() {

  // create all subscribers
//  ros::Subscriber<std_msgs::Float32> time_sync("/vehicle/dbw/time_sync", time_sync_callback);
  ros::Subscriber<std_msgs::Float32> dvel_sub("/vehicle/dbw/desired_velocity", desired_vel_callback);
  ros::Subscriber<std_msgs::Float32> cvel_sub("/vehicle/dbw/current_velocity", current_vel_callback);

  ros::Subscriber<std_msgs::Bool> go_sub("/vehicle/dbw/go", set_go_status);
  ros::Subscriber<std_msgs::Float32> sub2("/sensor/joystick/right_stick_y", joystick_callback);
  ros::Subscriber<std_msgs::Bool> sub3("/sensor/joystick/enabled", joystick_enabled_callback);
//  ros::Subscriber<std_msgs::Float32> dir_accel_sub("/sensor/dbw/remote_velocity", dir_accel_callback);

  nh.initNode();

  // subscribe to all topics
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(cvel_sub);
  nh.subscribe(dvel_sub);
  nh.advertise(kill_pub);
//  nh.subscribe(dir_accel_sub);

  nh.advertise(pos_pub);

  // RC receiver code
  pinMode(2, INPUT); 
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(8, INPUT);

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

  // Read the pulse width of
//  ch1 = pulseIn(2, HIGH, 25000);  
//  ch2 = pulseIn(3, HIGH, 25000); 
//  ch3 = pulseIn(4, HIGH, 25000);
//  ch4 = pulseIn(5, HIGH, 25000); 
//  ch5 = pulseIn(8, HIGH, 25000);

//  if (ch5 >= 1600) {
//    kill_msg.data = true;
//    killed = true;
//  } else {
//    killed = false;
//    kill_msg.data = false;
//  }

  kill_pub.publish(&kill_msg);

  // ==========================
  if (killed)
    actuator_target_pos = LA_MIN;
  
  // accelerator control
  if (joystick_enabled && !killed) {
    potWrite(slave_Select_Pin, B00010001, cmd_val);
    potWrite(slave_Select_Pin, B00010010, cmd_val);
  } else if (!joystick_enabled && !killed) {
    potWrite(slave_Select_Pin, B00010001, remote_accel_val);
    potWrite(slave_Select_Pin, B00010010, remote_accel_val);
  } else if (killed) {
    potWrite(slave_Select_Pin, B00010001, POT_MIN);
    potWrite(slave_Select_Pin, B00010010, POT_MIN);
  }

  // linear actuator brake control
  if (abs(actuator_pos - actuator_target_pos) > 10) {
    if (actuator_pos < actuator_target_pos)
      move_actuator(255, 0);
    else if (actuator_pos > actuator_target_pos)
      move_actuator(255, 1);
  }
  else
    stop_actuator();

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
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
