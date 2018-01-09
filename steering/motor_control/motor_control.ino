volatile unsigned int count; //count for encoder

String msg; //Full serial message
int len; //Total length of message
boolean proceed; //Whether should steer

float steering_value; //steering value
boolean dir; //steering direction


int PWMA = 7; //Speed control
int AIN1 = 6; //Direction
int AIN2 = 5; //Direction
int STBY = 4; //Standby

//function when receive encoder interrupt
void onDetect(){
  count++;
}

void setup(){
  Serial.begin(9600);
  //setup encoder
  attachInterrupt(0, onDetect, RISING);
  //setup motor
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  //initialize
  count = 0;
  msg = "";
  next = "";
  len = 7;
  steering_value = 0.0;
  dir = 0;
}

//turn encoder count to radian value
float getRadian(int c){
  return (float)c * 2 * 3.14159265359 / 420;
}

//check if is beginning of message
boolean isBegin(){
  return Serial.available && Serial.read() == "b";
}

//check if is end of message
boolean isEnd(){
  return Serial.available && Serial.read() == "e";
}

void clear(){
  count = 0; 
  while(Serial.available) Serial.read(); // clear buffer
}

void loop(){
  if (isBegin()){
      msg = "";
      //message body
      for(int i = 0; i < len-2; i++){
        proceed = true;
        if(Serial.available()) msg += Serial.read();
        else {
          proceed = false;
          break; 
        }
      }
      //steering
      if(proceed && isEnd()){
        Serial.println(msg);
        steering_value = msg.toInt();
        if (steering_value > 0) dir = 0; else dir = 1;
        for(int i = 0; i < 40; i++){
          if(getRadian(count) < abs(steering_value)) {
            move(255,dir);
            delay(1);
          } else break;
        }
      }
  } else {
    move(0,!dir);
    delay(1);
  }
  clear(); 
}

void move(int spd, boolean dir){
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

