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

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;


void servo_cb( const std_msgs::UInt16& cmd_msg) {
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  digitalWrite(13, HIGH - digitalRead(13)); //toggle led
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() {
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo.attach(9); //attach it to pin 9
}

void loop() {
  nh.spinOnce();
  delay(1);
}

#include <SoftwareSerial.h>

#define RPWM 7
#define LPWM 6
#define LEN 6 //length of the actual message
// #define FPS 1
#define M_PI 3.14159265359
#define THRESHOLD 0.5

SoftwareSerial mySerial(10, 11); // RX, TX

unsigned long count; //count for encode

char msg[LEN]; //actual message
char pos_msg[LEN];
byte *send_msg;
double pos = 0.0; //steering position
unsigned long prev_t; //previous time

float steering_value; //steering value
boolean dir; //steering direction

//function when receive encoder interrupt
const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital

void encoderPinChangeA() {
  count += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  count += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}

void setup() {

  Serial.begin(115200);
  mySerial.begin(115200);
  //setup encoder
  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);
  //setup motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pos = 0.0;
  prev_t = 0;
  dir = 0;
  clr();
}

//turn encoder count to radian value
double getRadian(int c) {
  return (double)c * 0.6428571429 * (M_PI / 180.0);
}

void clr() {

  steering_value = 0.0;
  for (int i = 0; i < LEN; i++) {
    msg[i] = '?';
  }

}

void debug_comm() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.print("char:");
    Serial.print(c);
    Serial.print(" begin:");
    Serial.print(c == 'b');
    Serial.print(" end:");
    Serial.println(c == 'e');
  }
}

void debug_motor() {
  delay(1000);
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(1000);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 255);
  delay(1000);//
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(1000);
}

void loop() {

  //debug_comm();

  if (mySerial.read() == 'b') {
    Serial.println("Begin");
    mySerial.readBytes(msg, LEN);
    Serial.println(msg);
    if (mySerial.read() == 'e') {
      Serial.println("End");
      steering_value = atof(msg);
      Serial.print("Steering Value: "); Serial.println(steering_value);

      //actuation
      if (int(steering_value) > 0) {
        dir = 0;
        if (steering_value > 8 * M_PI) steering_value = 8 * M_PI;
      } else {
        dir = 1;
        if (steering_value < -8 * M_PI) steering_value = -8 * M_PI;
      }

      if (abs(steering_value - pos) > THRESHOLD) {
        Serial.println("turning...");
        while (1) {

          if (dir == 0) {
            if (getRadian(count) > steering_value) {
              Serial.print("current: "); Serial.println(getRadian(count));
              break;
            } else {
              mv(255, dir);
            }
          } else {
            if (getRadian(count) < steering_value) {
              Serial.print("current: "); Serial.println(getRadian(count));
              break;
            } else {
              mv(255, dir);
            }
          }

        }
        pos = getRadian(count);
      }
      Serial.print("position: ");
      Serial.println(pos);
      Serial.print("count: ");
      Serial.println(count);
    }
  }
  st();
  clr();
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

