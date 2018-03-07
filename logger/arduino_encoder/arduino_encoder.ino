// Red - 5V
// Black - GND
#include <Wire.h>

#define SLAVE_ADDRESS 0x50
const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
int encoder = 0;

void setup() {
  
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  pinMode(encoder_a, INPUT_PULLUP);
  pinMode(encoder_b, INPUT_PULLUP);
  Wire.onRequest(sendData);
  attachInterrupt(0, encoderPinChangeA, CHANGE);
  attachInterrupt(1, encoderPinChangeB, CHANGE);
  // Wire.beginTransmission(0);
  Serial.println("Ready!");
}

void loop() {
  
//  double angle = encoder; //0.00261799* / 4;
//  sendData(angle);
}

void sendData() {
  String str = String(encoder);
  char arr[str.length() + 1];
  str.toCharArray(arr, str.length() + 1);
  Wire.write(arr);
  Serial.println(encoder);
  Serial.println(arr);
}

void encoderPinChangeA() {
  encoder += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB() {
  encoder += digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}
