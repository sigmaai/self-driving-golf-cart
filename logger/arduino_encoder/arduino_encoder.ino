// Red - 5V
// Black - GND
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>


const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
int encoder = 0;

ros::NodeHandle  nh;

std_msgs::Float32 steering_msg;
ros::Publisher pub_steering("/data_logger/steering_angle", &steering_msg);

void setup() {
  
  Serial.begin(9600); // start serial for output
  
  pinMode(encoder_a, INPUT_PULLUP);
  pinMode(encoder_b, INPUT_PULLUP);

  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);

  nh.initNode();
  nh.advertise(pub_steering);

  Serial.println("Ready!");
}

void loop() {
  
   steering_msg.data = encoder;
   pub_steering.publish(&steering_msg);
   nh.spinOnce();
   delay(10);
}

void encoderPinChangeA() {
  encoder += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  encoder += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}
