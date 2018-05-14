#include <SparkFun_TB6612.h>

int pinAIN1 = 22;
int pinAIN2 = 23;
int pinPWMA = 8;
int pinSTBY = 53;

String stringtemp = "";

int s0 = 32;
int s1 = 33;
int s2 = 34;
int s3 = 35;

int SIG_pin = 8;
Motor motor1 = Motor(pinAIN1, pinAIN2, pinPWMA, 1, pinSTBY);
void setup() {
  // put your setup code here, to run once:
Serial.begin(500000);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
    pinMode(pinSTBY, OUTPUT);

  pinMode(9, OUTPUT);
    
Serial.println("arduino starts");
Serial.setTimeout(5000);


}

void loop() {
  // put your main code here, to run repeatedly:
 motor1.drive(255);

  for(int i =0;i<12;i++){
    long stime, etime;
    
    writeMux(i);
    stime = micros();

    etime = micros();
    
    
    
    Serial.print(" TIME : ");
    Serial.println(etime-stime);
   delay(1000);
  }

//  if(Serial.available() >0){
//    stringtemp = Serial.readStringUntil('$');
//    Serial.println(stringtemp);
//  }


//motor1.drive(255,1000);
  
}

int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0},
    {1,0,0,0},
    {0,1,0,0},
    {1,1,0,0},
    {0,0,1,0},
    {1,0,1,0},
    {0,1,1,0},
    {1,1,1,0},
    {0,0,0,1},
    {1,0,0,1},
    {0,1,0,1},
    {1,1,0,1},
    {0,0,1,1},
    {1,0,1,1},
    {0,1,1,1},
    {1,1,1,1}
  };
  for(int i =0; i<4;i++){
    digitalWrite(controlPin[i],muxChannel[channel][i]);
  }
  int val = analogRead(SIG_pin);
  return val;
}
void writeMux(int channel){
   int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0},
    {1,0,0,0},
    {0,1,0,0},
    {1,1,0,0},
    {0,0,1,0},
    {1,0,1,0},
    {0,1,1,0},
    {1,1,1,0},
    {0,0,0,1},
    {1,0,0,1},
    {0,1,0,1},
    {1,1,0,1},
    {0,0,1,1},
    {1,0,1,1},
    {0,1,1,1},
    {1,1,1,1}
  };
  for(int i =0; i<4;i++){
    digitalWrite(controlPin[i],muxChannel[channel][i]);
  }

}

