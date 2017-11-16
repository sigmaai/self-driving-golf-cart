

#include <Wire.h>

void setup(){
  Wire.begin(); //Join i2c bus (address optional for master)
  Serial.begin(9600);
}

byte val = 0;

void loop(){
  Wire.beginTransmission(40); //Transmit to device #40
                              //Device Address is specified in Datasheet
  Wire.write(byte(0xaa));     //Sends Instruction Byte
  Wire.write(val);            //Sends Potentiometer Value Byte
  Wire.endTransmission();     //Stop Transmitting
  
  val++;                      //Increment Value
  
  if(val == 256){             //If reached 256th Position (Max)
    val = 0;                  //Start over from lowest value
  }
  
  Serial.println(val);
  
  delay(250);
}

