volatile unsigned int count; 
float steering_value;
boolean dir;
//Motor A
int PWMA = 7; //Speed control
int AIN1 = 6; //Direction
int AIN2 = 5; //Direction
int STBY = 4; //Standby

void onDetect(){
  count++;
}

void setup(){
  Serial.begin(9600);
  attachInterrupt(0, onDetect, RISING);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  count = 0;
}

float getRadian(int c){
  return (float)c * 2 * 3.14159265359 / 420;
}

void loop(){
  while (Serial.available()){
    steering_value = Serial.readString().toInt();
    Serial.println(steering_value);
    if (steering_value > 0) dir = 0;
    else dir = 1;
    while(getRadian(count) < abs(steering_value)){
      move(255,dir);
    }
  }
  
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
  }

  digitalWrite(AIN1, inPin1);
  digitalWrite(AIN2, inPin2);
  analogWrite(PWMA, spd);
}

