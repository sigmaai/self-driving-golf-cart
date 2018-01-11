#define PWMA 7
#define AIN1 6
#define AIN2 5
#define STBY 4

#define LEN 5 //length of the actual message
#define FPS 1

volatile unsigned int count; //count for encoder

char msg[LEN]; //actual message 

float pos; //steering position
unsigned long prev_t; //previous time

float steering_value; //steering value
boolean dir; //steering direction

//function when receive encoder interrupt
void onDetect(){
  count++;
}

void setup(){
  Serial.begin(115200);
  //setup encoder
  attachInterrupt(0, onDetect, RISING);
  //setup motor
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pos = 0.0;
  prev_t = 0;
  dir = 0;
  clr();
}

//turn encoder count to radian value
float getRadian(int c){
  return (float)c * 2.0 * M_PI / 420.0;
}

void clr(){
  count = 0; 
  steering_value = 0.0;
  for (int i = 0; i < LEN; i++){
    msg[i] = '?'; 
  }
}

void debug(){
  while(Serial.available()){
    char c = Serial.read();
    Serial.print("char:");
    Serial.print(c);
    Serial.print(" begin:");
    Serial.print(c=='b');
    Serial.print(" end:");
    Serial.println(c=='e');
  }
}

void loop(){
  if(Serial.read() == 'b'){
    Serial.println("Begin");
    Serial.readBytes(msg,LEN);  
    Serial.println(msg);
    if (Serial.read() == 'e') {
      Serial.println("End"); 
      steering_value = atof(msg);
      Serial.print("Steering Value: ");
      Serial.println(steering_value);
      if (steering_value > 0) dir = 0;
      else dir = 1;
      prev_t = millis();
      while(getRadian(count) < abs(steering_value) && pos < 2 * M_PI && (millis()-prev_t) < 1000/FPS) mv(255,dir);
      if (dir) pos += getRadian(count);
      else pos -= getRadian(count);
    }
  }
  st();
  clr();
}

void mv(int spd, boolean dir){
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

void st(){
  digitalWrite(STBY,LOW); 
  delay(10);
}
