
//Arduino pro mini with 4 motors - Slave code
//cannot use Serial class

#include <CapPin.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>

#define SLAVE 8  // slave number 
CapPin cPin_1 = CapPin(A2);
CapPin cPin_2 = CapPin(A3);
CapPin cPin_3 = CapPin(2);
CapPin cPin_4 = CapPin(3);
//float smoothed_1;
//float smoothed_2;
//float smoothed_3;
//float smoothed_4;
long total_1;
long total_2;
long total_3;
long total_4;

int pos[4] = {0,0,0,0};
int des[4] = {0,0,0,0};

long timeRev=0;
long timeRev2=0;
int receiveValue;

//  motordriver 1
const int pinAIN1_1 = 1;
const int pinAIN2_1 = 0;
const int pinPWMA_1 = 5;
const int pinSTBY_1 = 100;
const int pinBIN1_1 = 4;
const int pinBIN2_1 = 7;
const int pinPWMB_1 = 6;

// motordriver 2
const int pinAIN1_2 = 8;
const int pinAIN2_2 = 11;
const int pinPWMA_2 = 9;
const int pinSTBY_2 = 100;
const int pinBIN1_2 = 12;
const int pinBIN2_2 = 13;
const int pinPWMB_2 = 10;

// 4 motors setting
Motor motor1 = Motor(pinAIN1_1, pinAIN2_1, pinPWMA_1, 1, pinSTBY_1);
Motor motor2 = Motor(pinBIN1_1, pinBIN2_1, pinPWMB_1, 1, pinSTBY_1);
Motor motor3 = Motor(pinAIN1_2, pinAIN2_2, pinPWMA_2, 1, pinSTBY_2);
Motor motor4 = Motor(pinBIN1_2, pinBIN2_2, pinPWMB_2, 1, pinSTBY_2);
int counter = 0;
void setup() {
  // put your setup code here, to run once:

//LED_CHECK();
Wire.begin(SLAVE);
Wire.onReceive(receiveCallback);
Wire.onRequest(requestEvent);
//pinMode(LED_BUILTIN, OUTPUT);

Serial.begin(9600);

}

void loop() {      
//  Serial.print(counter++); Serial.print("\t");
//  Serial.print(pos[0]); Serial.print("\t");
//  Serial.print(pos[1]); Serial.print("\t");
//  Serial.print(pos[2]); Serial.print("\t");
//  Serial.println(pos[3]);
//
//  Serial.print(des[0]); Serial.print("\t");
//  Serial.print(des[1]); Serial.print("\t");
//  Serial.print(des[2]); Serial.print("\t");
//  Serial.println(des[3]);
long time1=millis();
 total_1 = cPin_1.readPin(2000);
 total_2 = cPin_2.readPin(2000);
 total_3 = cPin_3.readPin(2000);
 total_4 = cPin_4.readPin(2000);
//  Serial.print(total_1); Serial.print("\t");
//  Serial.print(total_2); Serial.print("\t");
//  Serial.print(total_3); Serial.print("\t");
//  Serial.println(total_4);

  pos[0] = analogRead(A0);
  pos[1] = analogRead(A1);
  pos[2] = analogRead(A6);
  pos[3] = analogRead(A7);

if(total_1>1000){
    motor1.drive(0);
  }else{
    Positioning(motor1,des[0]*4,pos[0]);
  }
if(total_2>1000){
    motor2.drive(0);
  }else{
    Positioning(motor2,des[1]*4,pos[1]);
  }
if(total_3>1000){
    motor3.drive(0);
  }else{
    Positioning(motor3,des[2]*4,pos[2]);
  }
if(total_4>1000){
    motor4.drive(0);
  }else{
    Positioning(motor4,des[3]*4,pos[3]);
  }
  long time2 = millis();
  Serial.println(time2-time1);
//    if(counter==0){
//      total_1 = cPin_1.readPin(2000);
////      Serial.print("\t");
////      Serial.print(total_1);
//      if(total_1>1000)
//        motor1.drive(0);
//      counter++;
//    }else if(counter==1){
//      total_2 = cPin_2.readPin(2000);
////      Serial.print("\t");
////      Serial.print(total_2); 
//      if(total_1>1000)
//        motor1.drive(0);
//        counter++;
//    }else if(counter==2){
//      total_3 = cPin_3.readPin(2000);
////      Serial.print("\t");
////      Serial.print(total_3);
//         if(total_1>1000)
//        motor1.drive(0);
//      counter++;
//    }else if(counter==3){
//      total_4 = cPin_4.readPin(2000);
////      Serial.print("\t");
////      Serial.println(total_4);  
//         if(total_1>1000)
//        motor1.drive(0);
//        counter=0;
//    }else{}
}

void receiveCallback(int aCount){
    if(aCount == 4){
    des[0] = Wire.read();
    des[1] = Wire.read();
    des[2] = Wire.read();
    des[3] = Wire.read();

    
    }else{
//      Serial.print("Unexpected number of bytes received: ");
//      Serial.println(aCount+"bytes came");
    }
}



//void LED_CHECK() { 
//  digitalWrite(LED_BUILTIN, HIGH);
//  delay(1000);
//  digitalWrite(LED_BUILTIN, LOW);
//}
void requestEvent() {
  // 자신의 슬레이브 주소를 담은 메세지를 마스터에게 보냅니다. 슬레이브 주소에 맞게 수정해야 합니다.
Wire.write("nooooo");
//  Wire.write(des[1]);
//  Wire.write(des[2]);
//  Wire.write(des[3]);   
}

void Positioning(Motor motorNum,int destination,int pos){
  int spd = abs(destination-pos);
  int dir = -(destination-pos)/abs(destination-pos);
  spd = map(spd,0,1023,75,255);
  spd = constrain(spd,75,255);
  spd = spd*dir;
  if(abs(pos-destination)>10){
    motorNum.drive(-spd);
  }else{
    motorNum.drive(0);
  }
//  delay(1);
}

void RandomTesting(){
  for(int counter = 0;counter<9;counter++){
      timeRev = millis();
      timeRev2 = millis();
      int randomPosition = random(50,950);
    while(timeRev2-timeRev<1000){
      pos[0] = analogRead(A0);
      pos[1] = analogRead(A1);
      pos[2] = analogRead(A6);
      pos[3] = analogRead(A7);
//      Positioning(motor1,randomPosition,pos[0]);
      Positioning(motor2,randomPosition,pos[1]);
      Positioning(motor3,randomPosition,pos[2]);
      Positioning(motor4,randomPosition,pos[3]);
      timeRev2 = millis();
    }
  }
}
int smooth(int data, float filterVal, float smoothedVal){
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .999999;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return (int)smoothedVal;
}

