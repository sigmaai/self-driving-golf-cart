// include the SPI library:
#include <SPI.h>
#include <Servo.h>

#define M_PI 3.14159265359
#define LEN 6
#define SPEED_MAX = 200
#define SPEED_MIN = 70

char msg[LEN]; //actual message

float cruise_speed; //cruise value

// set pin 10 as the slave select for the digital pot:
const int slave_Select_Pin  = 10;
int spd = 0;
int pos = 0;
Servo myservo;  // create servo object to control a servo

void setup() {

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);
  Serial.begin(115200);

  // initialize SPI:
  SPI.begin();
  myservo.attach(9);
  myservo.write(180);
  clr();
}

void clr() {

  for (int i = 0; i < LEN; i++) {
    msg[i] = '?';
  }
//  while (Serial.available()){
//    Serial.read();
//  }
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

//  debug_comm();

  if (Serial.read() == 'b') {
    Serial.println("Begin");
    Serial.readBytes(msg, LEN);
    Serial.println(msg);
    if (Serial.read() == 'a') {
      Serial.println("End");
      spd = atof(msg);
      cruise_speed = spd;
      delay(80); 
    }
  }
  Serial.println(cruise_speed);
  if (cruise_speed == -1){
    potWrite(slave_Select_Pin, B00010001, 0);
    potWrite(slave_Select_Pin, B00010010, 0);
    engage_break();
    Serial.println("break engaged");
    cruise_speed = 0;
  }else if (cruise_speed == 1){
    potWrite(slave_Select_Pin, B00010001, 60);
    potWrite(slave_Select_Pin, B00010010, 60);
  }else{
    potWrite(slave_Select_Pin, B00010001, 0);
    potWrite(slave_Select_Pin, B00010010, 0);
    myservo.write(180);
  }
  clr();
}

void engage_break(){
  
    for (pos = 180; pos >= 20; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
    if (pos == 20){
      for (int t = 200; t >= 0; t -= 1){
        myservo.write(20);
        delay(10);
      }
    }
  }
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




