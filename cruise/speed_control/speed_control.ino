// include the SPI library:
#include <SPI.h>
#include <Servo.h>

#define RPWM 7
#define LPWM 6
#define M_PI 3.14159265359
#define LEN 6
#define SPEED_MAX 100
#define SPEED_MIN 70
#define SCALAR 3.14159265359 / 2

unsigned long prev_t = 0; //previous time
volatile unsigned int count; //count for encode
char msg[LEN]; //actual message
boolean is_stopped = 0;

float cruise_speed; //cruise value

// set pin 10 as the slave select for the digital pot:
const int slave_Select_Pin  = 10;

void onDetect() {
  count++;
}

//turn encoder count to radian value
float getRadian(int c) {
  return (float)c * 2.0 * M_PI / 420.0;
}

void setup() {

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);
  attachInterrupt(0, onDetect, RISING);
  Serial.begin(115200);

  // initialize SPI:
  SPI.begin();
  clr();
}

void clr() {
  count = 0;
  cruise_speed = 0;
  for (int i = 0; i < LEN; i++) {
    msg[i] = '?';
  }
}

void debug_motor() {
  mv(128,0);
  delay(1000);
mv(128,1);
  delay(1000);//
mv(0,0);
  delay(1000);
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
//debug_comm();
//press_break(1);
//delay(1000);
//release_break(0.005);

//delay(1000);

  if (Serial.read() == 'b') {
    Serial.println("Begin");
    Serial.readBytes(msg, LEN);
    Serial.println(msg);
    if (Serial.read() == 'a') {
      Serial.println("End");
      cruise_speed = atof(msg);

      Serial.println(cruise_speed);
      if (cruise_speed == -1) {
        potWrite(slave_Select_Pin, B00010001, 0);
        potWrite(slave_Select_Pin, B00010010, 0);
        if (!is_stopped) press_break(1);
        Serial.println("brakes engaged");
      } else {
        potWrite(slave_Select_Pin, B00010001, 60);
        potWrite(slave_Select_Pin, B00010010, 60);
        if (is_stopped) release_break(1);
      }
    }
  }
  clr();
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

void press_break(float amount) {
  // remember to check for dir
  while (1) {
    mv(255, 0);
    //encoder
   //     Serial.println(count);
    if (count > 850 * amount) break;
  }
  is_stopped = 1;
  st();
  count = 0;

}

void release_break(float amount) {

  prev_t = millis();
  // remember to check for dir
  while (1) {
    mv(255, 1);
    //encoder
 //       Serial.println(count);
    if (count > 850 * amount) break;
  }
  is_stopped = 0;
  st();
  count = 0;
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
  analogWrite(LPWM,0);
  analogWrite(RPWM,0);
  delay(10);
}






