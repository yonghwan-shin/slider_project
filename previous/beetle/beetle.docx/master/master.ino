#include <Wire.h>

// 슬레이브 주소
//int slider_number = 100;
int SLAVE[100];


void setup() {
  for(int i =0; i<99; i++){
  SLAVE[i]=i+1;
}
  // Wire 라이브러리 초기화
  Wire.begin(); 
  // 직렬 통신 초기화
  Serial.begin(9600); 
}

void loop() {

  int input = 100;

  Wire.beginTransmission(SLAVE[3]);
  Wire.write(input >> 8);
  Wire.write(input & 255);
  Wire.endTransmission();
  time2 = micros();
  
  delay(10);
}

void i2c_communication(int i) {
  // 12 바이트 크기의 데이터 요청
  Wire.requestFrom(SLAVE[i], 12); 
  // 12 바이트 모두 출력할 때까지 반복
  for (int j = 0 ; j < 12 ; j++) {  
    // 수신 데이터 읽기
    char c = Wire.read(); 
    // 수신 데이터 출력
    Serial.print(c); 
  }
  Serial.println();
}
