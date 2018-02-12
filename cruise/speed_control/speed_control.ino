// include the SPI library:
#include <SPI.h>

#define M_PI 3.14159265359
#define LEN 6

float pid_p_gain = 1.3;               //Gain setting for the P-controller
float pid_i_gain = 0.04;              //Gain setting for the I-controller
float pid_d_gain = 18.0;              //Gain setting for the D-controller
float pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error, pid_error_temp;
const float PID_MAX;

char msg[LEN]; //actual message

float speed; //cruise value

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
      speed = atof(msg);
      if (speed > 0 && speed < 10) {
        potWrite(slave_Select_Pin, B00010001, speed);
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

void calculate_pid() {
  
  //speed up==========================================================
  pid_error_temp = gyro_input - pid_setpoint;
  pid_i_mem += pid_i_gain * pid_error_temp;
  if (pid_i_mem > PID_MAX)
    pid_i_mem = PID_MAX;
  else if (pid_i_mem < PID_MAX * -1)
    pid_i_mem = PID_MAX * -1;

  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > PID_MAX)
    pid_output = PID_MAX;
  else if (pid_output < PID_MAX * -1)
    pid_output = PID_MAX * -1;
    
  pid_last_d_error = pid_error_temp;

}



