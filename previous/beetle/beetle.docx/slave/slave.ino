#include <Wire.h>
#include <SparkFun_TB6612.h>
// 자신의 슬레이브 주소를 설정해 줍니다.(슬레이브 주소에 맞게 수정해야 합니다.)
#define SLAVE 4  

// motor1
const int pinAIN1 = 11;
const int pinAIN2 = 10;
const int pinPWMA = 9;
const int pinSTBY = A1;
Motor motor1 = Motor(pinAIN1, pinAIN2, pinPWMA, 1, pinSTBY);

// 카운터
int receiveValue;
long timeRev=0;
long timeRev2=0;
int pos = 0;


void setup() {
  // Wire 라이브러리 초기화
  // 슬레이브로 참여하기 위해서는 주소를 지정해야 한다.
  LED_CHECK();
  Wire.begin(SLAVE);
  Wire.onReceive(receiveCallback);
  // 마스터의 데이터 전송 요구가 있을 때 처리할 함수 등록
  Wire.onRequest(sendToMaster);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop () {
  RandomTesting(motor1);
}

void receiveCallback(int aCount){
  if(aCount == 2){
    receiveValue = Wire.read() <<8;
    receiveValue |= Wire.read();
    Serial.println(receiveValue);
  }else{
    Serial.print("Unexpected number of bytes received: ");
    Serial.println(aCount);
  }
}

void LED_CHECK() { 
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void sendToMaster() {
  // 자신의 슬레이브 주소를 담은 메세지를 마스터에게 보냅니다. 슬레이브 주소에 맞게 수정해야 합니다.
  Wire.write("4th Beetle    ");   
}

void Positioning(Motor motorNum,int destination){
  int spd = abs(destination-pos);
  int dir = -(destination-pos)/abs(destination-pos);
  spd = map(spd,0,1023,75,255);
  spd = constrain(spd,75,255);
  spd = spd*dir;
  if(abs(pos-destination)>10){
    motorNum.drive(-spd);
    digitalWrite(LED_BUILTIN, HIGH);
//    Serial.println(spd);
  }else{
    motorNum.drive(0);
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(5);
}
void RandomTesting(Motor motornum){
  for(int counter = 0;counter<9;counter++){
      timeRev = millis();
      timeRev2 = millis();
      int randomPosition = random(50,950);
    while(timeRev2-timeRev<1000){
      pos = analogRead(A0);
      Positioning(motornum,randomPosition);
      timeRev2 = millis();
    }
  }
}

