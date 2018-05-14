//Arduino pro mini with 4 motors - Slave code
//cannot use Serial class

#include <CapPin.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>

// slave number 
#define SLAVE 8

#define TOLERANCE 20
 
#define Pos_Pin_1 A2
#define Pos_Pin_2 A3
#define Pos_Pin_3 A6
#define Pos_Pin_4 A7

// Variables for PID control
#define Time 0.004;
double Kp = 0.5;
double Ki = 0.001;
double Kd = 0.00001;
double error[4];
double error_previous[4];
double P_control[4], I_control[4], D_control[4];
double PID_control[4];

//capacitive sensor
CapPin cPin_1 = CapPin(7);
CapPin cPin_2 = CapPin(8);
CapPin cPin_3 = CapPin(A0);
CapPin cPin_4 = CapPin(A1);
long total_cap[4] = {0, 0, 0, 0};
uint8_t total_send[4];

//potentiometer data
int pos[4] = {0, 0, 0, 0};
uint8_t pos_send[4];

//destination from MASTER
uint8_t des[4] = {0, 0, 0, 0};

long timeRev = 0;
long timeRev2 = 0;

//  motordriver 1             // motordriver 2
const int pinPWMA_1 = 3;      const int pinPWMA_2 = 6;
const int pinAIN2_1 = 1;      const int pinAIN2_2 = 13;
const int pinAIN1_1 = 0;      const int pinAIN1_2 = 12;
const int pinSTBY_1 = 100;    const int pinSTBY_2 = 100;
const int pinBIN1_1 = 2;      const int pinBIN1_2 = 9;
const int pinBIN2_1 = 4;      const int pinBIN2_2 = 10;
const int pinPWMB_1 = 5;      const int pinPWMB_2 = 11;

// 4 motors setting
Motor motor1 = Motor(pinAIN1_1, pinAIN2_1, pinPWMA_1, 1, pinSTBY_1);
Motor motor2 = Motor(pinBIN1_1, pinBIN2_1, pinPWMB_1, 1, pinSTBY_1);
Motor motor3 = Motor(pinAIN1_2, pinAIN2_2, pinPWMA_2, 1, pinSTBY_2);
Motor motor4 = Motor(pinBIN1_2, pinBIN2_2, pinPWMB_2, 1, pinSTBY_2);

void setup() {
  Wire.begin(SLAVE);
  //  Wire.setClock(100000);
  Wire.onReceive(receiveCallback);
  Wire.onRequest(requestEvent);
  //    Serial.begin(9600);
}

void loop() {
  getPos();
  //  checkCondition();
  PID_MOVE();
}

void receiveCallback(int aCount) {
  if (aCount == 4) {
    des[0] = Wire.read();
    des[1] = Wire.read();
    des[2] = Wire.read();
    des[3] = Wire.read();
  } else {
    //      Serial.print("Unexpected number of bytes received: ");
    //      Serial.println(aCount+"bytes came");
  }
}

void requestEvent() {
  //send capacitive sensor data - uint8_t
  total_send[0] = total_cap[0];
  total_send[1] = total_cap[1];
  total_send[2] = total_cap[2];
  total_send[3] = total_cap[3];
  Wire.write(total_send[0]);
  Wire.write(total_send[1]);
  Wire.write(total_send[2]);
  Wire.write(total_send[3]);

  //sned potentiometer data - uint8_t
  pos_send[0] = pos[0] / 4;
  pos_send[1] = pos[1] / 4;
  pos_send[2] = pos[2] / 4;
  pos_send[3] = pos[3] / 4;
  Wire.write(pos_send[0]);
  Wire.write(pos_send[1]);
  Wire.write(pos_send[2]);
  Wire.write(pos_send[3]);
}

void checkCondition() {
  Serial.print("SLAVE NUMBER "); Serial.print(SLAVE);  Serial.println("\t");
  Serial.print("DES"); Serial.print("\t");
  Serial.print(des[0]);  Serial.print("\t");
  Serial.print(des[1]);  Serial.print("\t");
  Serial.print(des[2]);  Serial.print("\t");
  Serial.print(des[3]);  Serial.println("\t");

  Serial.print("POS"); Serial.print("\t");
  Serial.print(pos[0]);  Serial.print("\t");
  Serial.print(pos[1]);  Serial.print("\t");
  Serial.print(pos[2]);  Serial.print("\t");
  Serial.print(pos[3]);  Serial.println("\t");

  Serial.print("CAP"); Serial.print("\t");
  Serial.print(total_send[0]);  Serial.print("\t");
  Serial.print(total_send[1]);  Serial.print("\t");
  Serial.print(total_send[2]);  Serial.print("\t");
  Serial.print(total_send[3]);  Serial.println("\t");  Serial.println("");
  delay(1000);
}
void pidcontrol(Motor motorNum, int motor_num, int destination, int pos) {
  error[motor_num] = destination - pos;
  P_control[motor_num] = Kp * error[motor_num];
  I_control[motor_num] += Ki * error[motor_num] * Time;
  D_control[motor_num] = Kd * (error[motor_num] - error_previous[motor_num]) / Time;
  PID_control[motor_num] = P_control[motor_num] + I_control[motor_num] + D_control[motor_num];
  int dir = PID_control[motor_num] / abs(PID_control[motor_num]);
  PID_control[motor_num] = abs(PID_control[motor_num]);
  PID_control[motor_num] = constrain(PID_control[motor_num], 80, 255);
  //  if(dir >0){
  //    PID_control[motor_num] +=10;
  //  }
  //tolerance setting
  if (abs(error[motor_num]) > TOLERANCE) {
    motorNum.drive(dir * PID_control[motor_num]);
  } else {
    motorNum.drive(0);
  }
  error_previous[motor_num] = error[motor_num];
}
//  Easy moving function
void Positioning(Motor motorNum, int destination, int pos) {
  int spd = abs(destination - pos);
  int dir = -(destination - pos) / abs(destination - pos);
  spd = map(spd, 0, 1023, 70, 255);
  spd = constrain(spd, 70, 255);
  spd = spd * dir;
  if (abs(pos - destination) > 20) {
    motorNum.drive(-spd);
  } else {
    motorNum.drive(0);
  }
  //  delay(1);
}

void PID_MOVE() {
  pidcontrol(motor1, 1, des[0] * 4, pos[0]);
  pidcontrol(motor2, 2, des[1] * 4, pos[1]);
  pidcontrol(motor3, 3, des[2] * 4, pos[2]);
  pidcontrol(motor4, 4, des[3] * 4, pos[3]);
}

void getPos() {
  total_cap[0] = cPin_1.readPin(2);
  total_cap[1] = cPin_2.readPin(2);
  total_cap[2] = cPin_3.readPin(2);
  total_cap[3] = cPin_4.readPin(2);
  pos[0] = analogRead(Pos_Pin_1);
  pos[1] = analogRead(Pos_Pin_2);
  pos[2] = analogRead(Pos_Pin_3);
  pos[3] = analogRead(Pos_Pin_4);
}

