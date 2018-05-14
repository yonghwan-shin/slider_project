#include <SparkFun_TB6612.h>

int pos = 0;
int des = 0;

long time1;
long time2;

const int pinAIN1 = 2;
const int pinAIN2 = 4;
const int pinPWMA = 3;
const int pinSTBY = 100; // We don't use standby

Motor motor1 = Motor(pinAIN1, pinAIN2, pinPWMA,1, pinSTBY);

void setup() {
  
  Serial.begin(9600);

}

void loop() {
  time1 = millis();
  
  pos = analogRead(A1);
//  Serial.print("POSITION : ");
//  Serial.print(pos);  Serial.print("\t");
//  Serial.print("DESTINATION : ");
//  Serial.print(des);  Serial.print("\t");
    
    while(Serial.available()>0){ 
      des = Serial.parseInt(); //put destination as SERIAL INPUT
    }
  Positioning(motor1,des,pos);
  
  time2 = millis();
//  Serial.print("Time : "); Serial.println(time2-time1);
}

void Positioning(Motor motorNum,int destination,int pos){
  int spd = abs(destination-pos);
  int dir = -(destination-pos)/abs(destination-pos);
  spd = map(spd,0,1023,75,255);
  spd = constrain(spd,75,255);
  spd = spd*dir;
  if( abs(pos-destination) > 10){ //change 
    motorNum.drive(-spd); 
  }else{
    motorNum.drive(0);
  }
  
  //  delay(1);
  
}
