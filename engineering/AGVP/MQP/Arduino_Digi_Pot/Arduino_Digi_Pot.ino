/* 
Wiring Configuration (Arduino Uno & DS1803-010 IC)
     Arduino GND -> GND Bus (Breadboard / Perf Board)
     Tie A2, A1, A0, and GND to GND (Pins 5, 6, 7, 8 to GND)
     Attach Arduino 5V to Chip VCC (Pin 16)
     Attach Arduino Analog 5 (SCLK) to Chip SCL (Pin 9)
     Attach Arduino Analog 4 (SDA) to Chip SDA (Pin 10)
     Measuring Resistance across High End of Resistor (H0) and Wiper Terminal of Resistor (W0) (Pins 14 and 12)

Operation
     Chip Defaults to >10k Ohms on Power-On w/ No I2C Signal
     Val Initiates to 142, which is 5k Ohms - Full Stop on Golf Cart Accelerator
     /Val Increments from 142 -> 255, which decrements the resistance proportionately from 5000 Ohms -> 404.5
     At Val = 255, it jumps to Val = 142, which is Full Speed back to Complete Stop
     Cycle Loops Indefinitely

Serial Monitor is Enabled on 9600 Baud Rate for Monitoring Value of Variable "val"

*/

#include <Wire.h>

byte val = 142;

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(0x28);
  Wire.write(B10101001);
  Wire.write(val);
  Wire.endTransmission();

  val++;
  
  if(val == 255){
    val = 145;
  }
  
  Serial.println(val);
  delay(100);
}
