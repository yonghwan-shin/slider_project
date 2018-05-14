
#include <SparkFun_TB6612.h>

/*
   Pins on the side are linear pots
   Wire pin 1 (ground), pin 2 (analog), pin 3 (5V). There are two sets: top and bottom
   They are separately wired, so you got to do all three pins from one set to get it to work
   They report almost identical data (<1% different). Range is 0 (near motor) to 1023 (far)

   Pin T is a cap sensor. Not yet checked. Try it with MPR121

   Motor pins TODO. Need a motor driver (easydriver is stepper only)
*/


// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 9
#define BIN1 11
#define AIN2 4
#define BIN2 7
#define PWMA 3
#define PWMB 5
#define STBY 10
double Kp = 0.3;
double Ki = 0.0001;
double Kd = 0.001;
double error;
double error_previous;
double current_pos;

double P,I,D;
double Time = 0.004;
double PID;
  unsigned long prev_time = 0;

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {

  // read the input on analog pin 0:
  double sensorValue1 = analogRead(A0);
  double sensorValue2 = analogRead(A1);
  // print out the value you read:
  //Serial.print  (sensorValue1);
  //Serial.print  ("  ");
  //Serial.println (sensorValue2);
  //delay(10);        // delay in between reads for stability
go(motor1,500,1000);
   
//
//    int d = sensorValue1 - sensorValue2;
//
//    if (abs(d) > 10)
//     go2(motor2,sensorValue1,10);

  

}

void go(Motor M, double destination,unsigned long duration) {
  
      unsigned long start = millis();
      while(millis()-start<duration){
        double sensorValue1 = analogRead(A0);
        error = destination - sensorValue1;
 
        P = error * Kp;
        I = I + Ki*error*Time;
        D = Kd*(error-error_previous)/Time;
        PID=P+I+D;
        PID = -(PID/abs(PID)) * (abs(PID)/2+65);
         if( abs(error) <10)
          PID = 0;
        
        M.drive(PID,2);
       
        Serial.print(sensorValue1);Serial.print("      "); Serial.println(PID);
        error_previous = error;
        
      }
  
  }
  

