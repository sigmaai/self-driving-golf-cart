#define RPWM 7
#define LPWM 6
#define LEN 6 //length of the actual message
#define FPS 1

#define M_PI 3.14159265359
#define THRESHOLD 0.3


volatile unsigned int count; //count for encode
volatile float rad;

char msg[LEN]; //actual message
char pos_msg[LEN];
byte *send_msg;
float pos = 0.0; //steering position
unsigned long prev_t; //previous time

float steering_value; //steering value
boolean dir; //steering direction

//function when receive encoder interrupt
void onDetect() {
  count++;
}

void setup() {
  Serial.begin(115200);
  //setup encoder
  attachInterrupt(0, onDetect, RISING);
  //setup motor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pos = 0.0;
  prev_t = 0;
  dir = 0;
  clr();
}

//turn encoder count to radian value
float getRadian(int c) {
  return (float)c * 2.0 * M_PI / 420.0;
}

void clr() {
  rad = 0;
  steering_value = 0.0;
  for (int i = 0; i < LEN; i++) {
    msg[i] = '?';
  }

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

void debug_motor() {
  delay(1000);
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(1000);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 255);
  delay(1000);//
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(1000);
}

void loop() {
  if (Serial.peek() == 'b') {
    Serial.read();
    //        Serial.println("Begin");
    Serial.readBytes(msg, LEN);
    //          Serial.println(msg);
    if (Serial.read() == 'e') {
      //              Serial.println("End");
      steering_value = atof(msg);
      //               Serial.print("Steering Value: "); Serial.println(steering_value);

      if ( abs(steering_value - pos) > THRESHOLD) {
        //actuation
        if ((steering_value - pos) > 0) {
          dir = 0;
          if ((steering_value - pos) > 4 * M_PI) steering_value = 4 * M_PI;
        } else {
          dir = 1;
          if ((steering_value - pos) < -4 * M_PI) steering_value = -4 * M_PI;
        }

        //time limit
        prev_t = millis();
        while ((millis() - prev_t) < 1000 / FPS) {
          mv(255, dir);
          //encoder
          if (getRadian(count) > abs(steering_value - pos)) break;
        }
        if (dir) pos -= getRadian(count);
        else pos += getRadian(count);
        count = 0;

      }
      dtostrf(pos, 4, 2, pos_msg);
      pos_msg[4] = '\n';
      Serial.write(pos_msg);
    }
  }
  st();
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
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  delay(10);
}

