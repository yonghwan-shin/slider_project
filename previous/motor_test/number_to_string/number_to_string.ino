/* Number to String
 * 주어진 숫자를 사용하여 문자열을 만든다
 * 
 */

void setup() {
  
  Serial.begin(9600);

}
  char buffer[12];
  
void loop() {

  long value = 12345;
  ltoa(value, buffer, 10);
  Serial.print( value);
  Serial.print ( " has  ");
  Serial.print(strlen(buffer));
  Serial.println(" digits");
  value = 123456789;

  ltoa(value, buffer, 10);
  Serial.print( value);
  Serial.print ( " has  ");
  Serial.print(strlen(buffer));
  Serial.println(" digits");
  delay(1000);
}
