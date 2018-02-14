// include the SPI library:
#include <SPI.h>
#include <Servo.h>

#define M_PI 3.14159265359
#define LEN 6

char msg[LEN]; //actual message

float cruise_speed; //cruise value

// set pin 10 as the slave select for the digital pot:
const int slave_Select_Pin  = 10;

Servo myservo;  // create servo object to control a servo

void setup() {

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);
  Serial.begin(9600);

  // initialize SPI:
  SPI.begin();
  myservo.attach(9);

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
      speed = atof(msg);
      if (speed > 0 && speed < 10) {
        cruise_speed = speed
        delay(80); 
      }
    }
  }
  if (cruise_speed == -1){
    engage_break();
  }
}

void engage_break(){
  myservo.write(20);
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




