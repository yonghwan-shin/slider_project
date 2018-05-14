// Arduino Due - Master code
//  takes 500 microsecond for each slave
#include <Wire.h>
  
int SLAVE[100];
long time1=0;
long time2=0;
int total1;
int total2;
int total3;
int total4;


 uint8_t i_1;
 uint8_t i_2;
 uint8_t i_3;
 uint8_t i_4;
void setup() {
//    for(int i =0; i<100; i++){
//    SLAVE[i]=i+1;
//    }
  Wire.begin();
  
  Serial.begin(9600);
}


uint8_t input = 0;
void loop() {
  
  input = random(0,254);
  
  time1 = micros();

  sendPosition(8,input,input,input,input);
  
//  Wire.requestFrom(8, 8);    // request 6 bytes from slave device #8
checkSlave(8);
//  time2 = micros();
//  long time12 = time2 - time1;
//  Serial.println(time12);


  delay(1000);

}
void sendPosition(int SLAVE_num, uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4){
  Wire.beginTransmission(SLAVE_num);
  Wire.write(motor1);
  Wire.write(motor2);
  Wire.write(motor3);
  Wire.write(motor4);
  Wire.endTransmission();
}
void checkSlave(int Slave_num){
    Serial.println("");
    Wire.requestFrom(Slave_num,8);
//  while(Wire.available()){
//    char c = Wire.read();
//    Serial.print(c);}
 total1 = Wire.read();
 total2 = Wire.read();
 total3 = Wire.read();
 total4 = Wire.read();
 i_1 = Wire.read();
 i_2 = Wire.read();
 i_3 = Wire.read();
 i_4 = Wire.read();
Serial.print(total1);Serial.print("\t");
Serial.print(total2);Serial.print("\t");
Serial.print(total3);Serial.print("\t");
Serial.print(total4);Serial.print("\t");

Serial.print(i_1);Serial.print("\t");
Serial.print(i_2);Serial.print("\t");
Serial.print(i_3);Serial.print("\t");
Serial.print(i_4);Serial.print("\t");
  
}

