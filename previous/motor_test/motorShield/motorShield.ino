#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS_0 = Adafruit_MotorShield(0x60); 
Adafruit_MotorShield AFMS_1 = Adafruit_MotorShield(0x61); 
Adafruit_DCMotor *Motor1 = AFMS_0.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS_0.getMotor(2);
Adafruit_DCMotor *Motor3 = AFMS_0.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS_0.getMotor(4);
Adafruit_DCMotor *Motor5 = AFMS_1.getMotor(1);
Adafruit_DCMotor *Motor6 = AFMS_1.getMotor(2);
Adafruit_DCMotor *Motor7 = AFMS_1.getMotor(3);
Adafruit_DCMotor *Motor8 = AFMS_1.getMotor(4);

struct MUX{
  int s0;
  int s1;
  int s2;
  int s3;
  int SIG_pin;
};

// pin for MUX

 MUX mux1 = {6,7,8,9,A10};
 MUX mux2 = {2,3,4,5,A10};

int PositionData[128];

void setup() {
  
  // MUX setting
  setMux(mux1);
  setMux(mux2);
  
  
Serial.begin(57600);
Serial.flush();

AFMS_0.begin();
  Serial.println("MotorShield...0...ready");
AFMS_1.begin();
  Serial.println("MotorShield...1...ready");

delay(1000);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

}

void loop() {


// for 128 sliders
int counter = 0;
for(int i = 0; i<16;i++){
    for(int j=0; j<8;j++){
      ChangeMux(mux2,j);
      PositionData[counter] = readMux(mux1,i);
      Positioning(Motor4,500,PositionData[counter++]);
    }
  }

}

void Positioning(Adafruit_DCMotor *motornumber, int pos,int channel){
   int val = channel;
   
   int spd = abs(val - pos);
  int dir = (val-pos)/abs(val-pos);
  spd = map(spd,0,1023,60,200);
  spd = constrain(spd,60,200);
  spd = spd*dir;
  
  if(abs(val-pos)>10){
    
      if(val-pos < 0){
        motornumber->run(FORWARD);
      }else{
        motornumber->run(BACKWARD);
      }
      motornumber->setSpeed(spd);
//    Serial.println(spd);
  }else{
    motornumber->run(RELEASE);
//    Serial.println("0");
  }
}

//    MUX functions
int readMux(MUX mux_num, int channel){
  int controlPin[] = {mux_num.s0, mux_num.s1, mux_num.s2, mux_num.s3};

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
  int val = analogRead(mux_num.SIG_pin);
  return val;
}

void ChangeMux(MUX mux_num, int channel){
  int controlPin[] = {mux_num.s0, mux_num.s1, mux_num.s2, mux_num.s3};

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

void setMux(MUX mux_num){
  pinMode(mux_num.s0, OUTPUT); 
  pinMode(mux_num.s1, OUTPUT); 
  pinMode(mux_num.s2, OUTPUT); 
  pinMode(mux_num.s3, OUTPUT); 

  digitalWrite(mux_num.s0, LOW);
  digitalWrite(mux_num.s1, LOW);
  digitalWrite(mux_num.s2, LOW);
  digitalWrite(mux_num.s3, LOW);
}

