// Arduino Due - Master code

#include <Wire.h>
#include <Metro.h>


Metro metro0 = Metro(1000);   // broadcast interval

const int NUM_OF_SLAVES = 2;
uint8_t input[NUM_OF_SLAVES][4];
unsigned long time1 = 0;
unsigned long time2 = 0;

int total1[NUM_OF_SLAVES];
int total2[NUM_OF_SLAVES];
int total3[NUM_OF_SLAVES];
int total4[NUM_OF_SLAVES];
uint8_t i_1[NUM_OF_SLAVES];
uint8_t i_2[NUM_OF_SLAVES];
uint8_t i_3[NUM_OF_SLAVES];
uint8_t i_4[NUM_OF_SLAVES];

void setup() {
  Wire.begin();
  //  Wire.setClock(100000);
  Serial.begin(9600);
}

void loop() {

//    if (metro0.check() == 1) {
//    
//    }

  time1 = micros();
  input[0][0] = random(0, 254);
  input[0][1] = random(0, 254);
  input[0][2] = random(0, 254);
  input[0][3] = random(0, 254);
  input[1][0] = random(0, 254);
  input[1][1] = random(0, 254);
  input[1][2] = random(0, 254);
  input[1][3] = random(0, 254);

  sendPosition(7, input[0][0], input[0][1], input[0][2], input[0][3]);
  sendPosition(8, input[1][0], input[1][1], input[1][2], input[1][3]);
  checkSlave(7);
  checkSlave(8);
  time2 = micros();
  showSlave(7);
  showSlave(8);

  Serial.print("time : "); Serial.print(time2 - time1); Serial.println(" microsecond");
//  delay(500);
}
void sendPosition(int SLAVE_num, uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4) {
  Wire.beginTransmission(SLAVE_num);
  Wire.write(motor1);
  Wire.write(motor2);
  Wire.write(motor3);
  Wire.write(motor4);
  Wire.endTransmission();
}
void showSlave(int Slave_num) {
  Serial.println("");
  Serial.print("Slave "); Serial.print(Slave_num); Serial.print(" -> ");
  Serial.print(total1[Slave_num - 1]); Serial.print("\t");
  Serial.print(total2[Slave_num - 1]); Serial.print("\t");
  Serial.print(total3[Slave_num - 1]); Serial.print("\t");
  Serial.print(total4[Slave_num - 1]); Serial.print("\t");
  Serial.print(" ||"); Serial.print("\t");
  Serial.print(i_1[Slave_num - 1]); Serial.print("\t");
  Serial.print(i_2[Slave_num - 1]); Serial.print("\t");
  Serial.print(i_3[Slave_num - 1]); Serial.print("\t");
  Serial.print(i_4[Slave_num - 1]); Serial.print("\t");
}

void checkSlave(int Slave_num) {
  Wire.requestFrom(Slave_num, 8);
  total1[Slave_num - 1] = Wire.read();
  total2[Slave_num - 1] = Wire.read();
  total3[Slave_num - 1] = Wire.read();
  total4[Slave_num - 1] = Wire.read();
  i_1[Slave_num - 1] = Wire.read();
  i_2[Slave_num - 1] = Wire.read();
  i_3[Slave_num - 1] = Wire.read();
  i_4[Slave_num - 1] = Wire.read();


}

