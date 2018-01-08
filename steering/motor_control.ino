volatile unsigned int count; 
unsigned long prev_time;
//Motor A
int PWMA = 7; //Speed control
int AIN1 = 6; //Direction
int AIN2 = 5; //Direction
int STBY = 4; //Standby

void onDetect(){
  count++;
  if (count%10==0){
  Serial.println(count);
  }
}

void setup(){
  Serial.begin(9600);
  attachInterrupt(0, onDetect, RISING);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  count = 0;
  prev_time = 0;
}

float getDegree(int c){
  return (float)c * 360 / 420;
}

void loop(){
}

void move(int spd, boolean dir){
//Move speed and direction
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise
  digitalWrite(STBY,HIGH);

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;
 
  if(dir){
      inPin1 = HIGH;
      inPin2 = LOW;
      Serial.println("reverse");
  }

  digitalWrite(AIN1, inPin1);
  digitalWrite(AIN2, inPin2);
  analogWrite(PWMA, spd);
}
