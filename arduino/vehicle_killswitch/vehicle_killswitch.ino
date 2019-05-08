//
// self-driving golf cart killswith by Neil Nie
// Functional safety | self driving golf cart
// (c) Yongyang Nie, All rights reserved
//

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

// declare the publisher
std_msgs::Bool kill_msg;
ros::Publisher kill_pub("/vehicle/safety/killed", &kill_msg);

std_msgs::Float32 steering_msg;
ros::Publisher steering_pub("/vehicle/dbw/steering_cmds/", &steering_msg);

std_msgs::Float32 accel_msg;
ros::Publisher accel_pub("/sensor/dbw/remote_velocity", &accel_msg);


//Setup routine
void setup(){

  nh.initNode();

  nh.advertise(kill_pub);
  nh.advertise(steering_pub);
  nh.advertise(accel_pub);

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11) to trigger an interrupt on state change

}

//void interrupt_method() {
//
//  kill_msg.data = true;
//}

//Main program loop

void loop(){

  if (receiver_input_channel_4 >= 1600) {
    kill_msg.data = true;
  }else{
    kill_msg.data = false;
  }

  kill_pub.publish(&kill_msg);

  float filtered_val = mapf(receiver_input_channel_3, 1040, 1890, -1, 1);
  float steering_val = mapf(filtered_val, -1, 1, -M_PI, M_PI);
  steering_msg.data = steering_val;
  steering_pub.publish(&steering_msg);

  float brake_val = 1.0 - mapf(receiver_input_channel_2, 1480, 1030, 1, 0);
  float accel_val = mapf(receiver_input_channel_1, 1045, 1890, 0, 1);

  if (brake_val > 0.05) {
    accel_msg.data = -1 * brake_val;
  }else{
    accel_msg.data = accel_val;
  }

  accel_pub.publish(&accel_msg);
  
  nh.spinOnce();

  delay(5);
} 

//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect){

  
  //Channel 1=========================================
  if(last_channel_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if(last_channel_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if(last_channel_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if(last_channel_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
