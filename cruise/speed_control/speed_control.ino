// include the SPI library:
#include <SPI.h>


#define M_PI 3.14159265359
#define LEN 6

char msg[LEN]; //actual message

float cruise_value; //cruise value

// set pin 10 as the slave select for the digital pot:
const int slave_Select_Pin  = 10;

void setup() {

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);
  Serial.begin(9600);

  // initialize SPI:
  SPI.begin();
//  potWrite(slave_Select_Pin, B00010001, level1);
//  potWrite(slave_Select_Pin, B00010010, level2);
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

void loop() {

 // debug_comm();

  if (Serial.read() == 'b') {
    Serial.println("Begin");
    Serial.readBytes(msg, LEN);
    Serial.println(msg);
    if (Serial.read() == 'e') {
      Serial.println("End");
      cruise_value = atof(msg);
      if (cruise_value > 0) {
        potWrite(slave_Select_Pin, B00010001, cruise_value);
        delay(80); 
      }
    }
  }
/*
  for (int i = 70; i <= 130; i++) {
    potWrite(slave_Select_Pin, B00010001, i);
    potWrite(slave_Select_Pin, B00010010, i);
    delay(80);
  }

  for (int i = 130; i >= 70;  i--) {
    potWrite(slave_Select_Pin, B00010001, i);
    potWrite(slave_Select_Pin, B00010010, i);
    delay(80);
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


