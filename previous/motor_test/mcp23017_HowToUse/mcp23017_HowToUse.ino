#include "Wire.h"

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x20);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x20);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.begin(115200);
}

void binaryCount() {

  for (byte a = 0; a < 256; a++) {
    Wire.beginTransmission(0x20);
    Wire.write(0x12);
    Wire.write(a);
    Wire.endTransmission();
    Wire.beginTransmission(0x20);
    Wire.write(0x13);
    Wire.write(a);
    Wire.endTransmission();
  }
}

void loop() {
  binaryCount();
  delay(500);

}
