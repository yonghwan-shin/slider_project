/* SplitSplit Sketch
 * 쉼표로 구분된 문자열을 분리한다.
 * 
 */
const int MAX_STRING_LEN = 20;
char stringList[] = "Peter,Paul,Mary";
char stringBuffer[MAX_STRING_LEN+1];

void setup() {
  Serial.begin(9600);
}

void loop() {
  char *str;
  char *p;
  strncpy(stringBuffer, stringList, MAX_STRING_LEN);
  Serial.println(stringBuffer);

    for(str = strtok_r(stringBuffer, "," , &p);
      str;
      str = strtok_r(NULL, ",", &p)
      )
    {
      Serial.println(str);
    }
    delay(2000);
}
