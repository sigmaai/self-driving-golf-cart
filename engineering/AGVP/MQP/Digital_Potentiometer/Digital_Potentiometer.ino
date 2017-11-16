#include <Wire.h>
 
void setup() {
  Wire.begin();
}
byte val = 0;
void loop() {
  Wire.beginTransmission(0x28);
  Wire.write(B10101001);
  Wire.write(val);
  Wire.endTransmission();
 
  val++;
  if(val == 128) val = 0;
  delay(500);
}
RAW Paste Data
