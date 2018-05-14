
#include <SparkFun_TB6612.h>

// motor1
int pinAIN1 = 22;
int pinAIN2 = 23;
int pinPWMA = 8;
// motor2
int pinBIN1 = 13;
int pinBIN2 = 7;
int pinPWMB = 5;
//stanby
int pinSTBY = 53;

//constants to remember the parameters
static boolean down = 0;
static boolean up = 1;
static int motor1 = 0;
static int motor2 = 1;

//define for PID control

double Kp = 0.33;
double Ki = 0.0001;
double Kd = 0.0001;

struct PID_control{

  double P;
  double I;
  double D;
  double error;
  double error_previous;
  double current_pos;
  double PID;
  double sensorValue;
};
//for 2 motor
  struct PID_control PIDcontrol[2];

double Time = 0.004;

unsigned long prev_time = 0;
String stringTemp = "";String stringX="";String stringY="";
int intTemp[2] = {0,0};


void setup() {
  Serial.begin(115200);
  //set PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  pinMode(pinSTBY, OUTPUT);

  Serial.setTimeout(10000);

}

void loop() {
//PIDcontrol[0].sensorValue = analogRead(A0);
//PIDcontrol[1].sensorValue = analogRead(A1);
//  if (Serial.available() > 0 ) {
//
//    stringTemp = Serial.readStringUntil('#');
//    int pos = stringTemp.indexOf(',');
//    
//    if(stringTemp[0] =='@'){
//      stringX = stringTemp.substring(1,pos);
//      stringY = stringTemp.substring(1+pos);
//      Serial.print(pos);
//      Serial.print("   ->  " +stringTemp+"  |  ");
//      Serial.print(stringX+"  |  ");
//      Serial.println(stringY);
//      intTemp[0] = stringX.toInt();
//      intTemp[1] = stringY.toInt();
//      
//      Serial.print("X:  ");
//      Serial.print(intTemp[0]);Serial.print("   |   ");
//      Serial.print("Y:  ");
//      Serial.println(intTemp[1]);
//    }
//        
//
//  }
//going(motor1,intTemp[0],PIDcontrol[0].sensorValue);
    

//   going(motor1,500,PIDcontrol[0].sensorValue);
  motorDrive(motor1,up,100);
  
//    Serial.println(PIDcontrol[0].sensorValue);



}





void motorDrive(int motorNumber, boolean motorDirection, int motorSpeed) {

  boolean pinIn1;

  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH

  if (motorDirection == down)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, !pinIn1);
    digitalWrite(pinAIN2, pinIn1);
    analogWrite(pinPWMA, motorSpeed);
  } else {
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);
    analogWrite(pinPWMB, motorSpeed);
  }
  //make STBY disabled
  digitalWrite(pinSTBY, HIGH);
}

void motorBrake(int motorNumber) {
  if (motorNumber == motor1)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
}

void motorStop(int motorNumber) {
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  } else {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  }
}

void motorsStanby() {
  digitalWrite(pinSTBY, LOW);
}

void going(int motornumber, double destination, double pos){
   
    PIDcontrol[motornumber].error = destination - pos;

    PIDcontrol[motornumber].P = PIDcontrol[motornumber].error * Kp;
    PIDcontrol[motornumber].I = PIDcontrol[motornumber].I + Ki * PIDcontrol[motornumber].error * Time;
    PIDcontrol[motornumber].D = Kd * (PIDcontrol[motornumber].error - PIDcontrol[motornumber].error_previous) / Time;
    PIDcontrol[motornumber].PID = PIDcontrol[motornumber].P + PIDcontrol[motornumber].I + PIDcontrol[motornumber].D;

    PIDcontrol[motornumber].PID = -(PIDcontrol[motornumber].PID / abs(PIDcontrol[motornumber].PID)) * (abs(PIDcontrol[motornumber].PID)+60);

    
    bool dir;
    if (PIDcontrol[motornumber].PID > 0)
    {
      dir = 0;
       PIDcontrol[motornumber].PID = PIDcontrol[motornumber].PID-10; 
    }
    else {
      dir = 1;
      PIDcontrol[motornumber].PID = PIDcontrol[motornumber].PID-20; 
    }
    
    if ( abs(PIDcontrol[motornumber].error) < 5){
      PIDcontrol[motornumber].PID = 0;
      
    }
    if( PIDcontrol[motornumber].PID >255){
      PIDcontrol[motornumber].PID = 255;
    }else if(PIDcontrol[motornumber].PID<-255){
      PIDcontrol[motornumber].PID = -255;
    }else{
      
    }
    motorDrive(motornumber, dir, abs(PIDcontrol[motornumber].PID));
//    delay(2);


//    Serial.print(PIDcontrol[motornumber].sensorValue); Serial.print("      "); Serial.println(PIDcontrol[motornumber].PID);
    PIDcontrol[motornumber].error_previous = PIDcontrol[motornumber].error;
  
}
