void setup() {
  
  Serial.begin(57600);
  Serial.flush();
  delay(1000);
  Serial.println("Labels,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11");

}

void loop() {
  Serial.print("Data,");
  for(int i=0; i<12; i++){
    Serial.print( analogRead(i));
    Serial.print(",");
  }
  Serial.print('\n');
  delay(1000);
}
