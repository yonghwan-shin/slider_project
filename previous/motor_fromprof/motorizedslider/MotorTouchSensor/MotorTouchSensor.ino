#include <SparkFun_TB6612.h>

/*
 * Pins on the side are linear pots
 * Wire pin 1 (ground), pin 2 (analog), pin 3 (5V). There are two sets: top and bottom 
 * They are separately wired, so you got to do all three pins from one set to get it to work
 * They report almost identical data (<1% different). Range is 0 (near motor) to 1023 (far)
 * 
 * Pin T is a cap sensor. Not yet checked. Try it with MPR121
 * 
 * Motor pins TODO. Need a motor driver (easydriver is stepper only)
 */


// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 4
#define BIN1 10
#define AIN2 5
#define BIN2 11
#define PWMA 3
#define PWMB 9
#define STBY 2

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// touch sensor
#include "mpr121.h"
#include "i2c.h"

// this does an analog read of a single MPR121 board for testing. Assumes the board is jumpered to ground (e.g. address 0xb4 and 0xb5)

// 1. Optimise sending of the raw data 
//    - change to raw bytes, no spacers. Look at lower bit sizes (e.g. 2/character?)              *** done & works ***
// 2. Optimise I2C calls - reading 48 registers seems to take around 15ms, or 3/regs per ms. 
//    - increasing bus speed??                                                                    *** done & works ***
//    - reducing period in delay_ms function                                                      *** done & works ***
//    - Try faster Arduino - might be non-bus processing
//    - Alter to do 'multi byte reading' for filt data - **IMPORTANT** to ensure both bytes match
//      - but doesn't seem to be a problem at the moment
//    - Reduce calls - try updating baselines less often? Need to verify this works with a given set of electrodes
//      - updating 4 baselines per time should be around 1ms and would get through all baselines in 12 cycles (2 r 3 times a second) 
// 3. Speed up MPR - like in beats. Not sure that will help                                       *** done & works - but no speed change ***
// 4. Change i2c lib for wire. This one is pretty wierd. Wire supports 2 byte reads...

// the hardware addresses for multiple boards from the manual
// 0x5A (manual - ADD->GRD) / 0b1011010 (7 bits) / 0b10110100 (8 bits, 0xB4 write) / 0b10110101 (8 bits, 0xB5, read)
// 0x5B (manual - ADD->3.3V) / 0b1011011 (7 bits) / 0b10110110 (8 bits, 0xB6 write) / 0b10110111 (8 bits, 0xB7, read)
// 0x5C (manual - ADD->SDA) / 0b1011100 (7 bits) / 0b10111000 (8 bits, 0xB8 write) / 0b10111001 (8 bits, 0xB9, read) 
// 0x5D (manual - ADD->SCL) / 0b1011101 (7 bits) / 0b10111010 (8 bits, 0xBA write) / 0b10111011 (8 bits, 0xBB, read)

// the hardware addresses translated into usable constants
// address 1 - works
#define MPR121_W_GRD    0xB4	
#define MPR121_R_GRD    0xB5

// address 2 - works
#define MPR121_W_PWR    0xB6	
#define MPR121_R_PWR    0xB7		

// address 3 - works
#define MPR121_W_SDA    0xB8	
#define MPR121_R_SDA    0xB9	

// address 4 - works
#define MPR121_W_SCL    0xBA	
#define MPR121_R_SCL    0xBB	

#define COUNT_START        '!'
#define COUNT_END          '@'

#define TIME_START         '@'
#define TIME_END           '#'

#define DATA_START         '#'
#define DATA_END           '$'

#define DATA_RAW_START     '%'
#define DATA_RAW_END       '&'

// timing variables - how frequently we poll the sensors and send to the serial port
unsigned long previousMillis = 0;
long interval = 10; // this would be 50/times second. Can slightly beat this :)

// a packet counter appended to each message sent - wraps to zero when it reaches "wrap"
int count = 0;
int wrap = 100;

#define BOARDS_USED        1 // how many boards are we using? 
#define SENSORS_PER_BOARD  12 // how many boards are we using? 
#define NUM_SENSORS        SENSORS_PER_BOARD * BOARDS_USED
// int id = 1; // use this if you just want one specific board.
#define NUM_BYTES          2

boolean sensorState[BOARDS_USED*SENSORS_PER_BOARD]; 

int base[BOARDS_USED*SENSORS_PER_BOARD];
int filt[BOARDS_USED*SENSORS_PER_BOARD];

/*
 * I was getting errors (weird behavior) with the built in % operator, so switched to my own modulus imlementation
 * Can't believe it. 
 */
int myMod(int a, int b)
  {return a - (b * int(a/b)); }
  
  
// state var for the two versions of the system - set useLarge to true to use 4.5mm board and false to use 3mm board
// this is needed because they have different PCB layouts - diff connections to the MPR121 boards and sensors. 
boolean useLarge = true;   
// when we just have one board
boolean useSolo = true;

// this state is toggled by a hardware button
const int buttonPin = 2;     // the number of the pushbutton pin
boolean lastRead = false;


void setup()
  {
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);     
  
  Serial.begin(115200);

  
  Serial.println("setting device up..");
  //DDRC |= 0b00010011;  // Pull-up on I2C Bus
  PORTC = 0b00110000;  // Pull-up on I2C Bus
  i2cInit();           // run the i2c init routine
  delay(100);          // wait a moment

  // run the sensor init routine - if you are using more boards, add them here. 
  for (int i=1;i<=BOARDS_USED;i++)
    {
      mpr121QuickConfig(getRead(i), getWrite(i));
    }

  delay(250);          // wait a moment
  // run the sensor init routine - if you are using more boards, add them here. 
  for (int i=1;i<=BOARDS_USED;i++)
    {
      read_MPR121_baseline(i);
    }
  }

// get read address of one of the sensor boards (1-4 are valid options)
byte getRead(int address)
  {
    switch (address) 
      {
        case 1 : return MPR121_R_GRD;
        case 2 : return MPR121_R_PWR;
        case 3 : return MPR121_R_SDA;
        case 4 : return MPR121_R_SCL;
      }  
    return 0;
  }
  
// get write address of one of the sensor boards (1-4 are valid options)
byte getWrite(int address)
  {
    switch (address) 
      {
        case 1 : return MPR121_W_GRD;
        case 2 : return MPR121_W_PWR;
        case 3 : return MPR121_W_SDA;
        case 4 : return MPR121_W_SCL;
      }  
    return 0;
  }  


// run continuously
void loop()
  {    
  processMotor();  

  //if (!useSolo)
    {
    boolean buttonState = digitalRead(buttonPin);  
    if (buttonState!=lastRead)
      {
      lastRead = buttonState;
      if (lastRead) // we've moved to true
        {
        useLarge = !useLarge;   
        if (useLarge)
          Serial.println("\n\nUsing LARGE (4.5mm) touch sensor\n");
        else
          Serial.println("\n\nUsing SMALL (3.0mm) touch sensor\n");
        }
      }
    }
    
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval)
    {   
    // read data    
    for (int i=1;i<=BOARDS_USED;i++)
      {
      Read_MPR121(i); // read binary
      read_MPR121_raw(i); // read thresholds and raw data
      }
    
    // send and process packet count
    Serial.print(COUNT_START); 
    count++;
    if (count>=wrap)
      count=0;
    Serial.print(count); 
    Serial.print(COUNT_END); 
    
    // send and process time. Doesn't need start and stop bytes
    Serial.print(currentMillis - previousMillis); 
    previousMillis = currentMillis;
    
    // send binary data to PC
    sendBinaryData();
    
    // sends the filtered and raw sensor values. 
    sendRawData();
    
    // finish the line
    Serial.println(); 
    
    // prints the binary sensor data into a grid (useful for checking mapping of sensors to grid positions)
    //printBinaryGrid();
    //delay(500); // pause for a bit
    }
}



void sendRawData()
  {
  Serial.print(DATA_RAW_START);     
  for (int i=0;i<NUM_SENSORS;i++)  
    {
    /*
    Serial.print(base[i]);
    Serial.print(",");
    Serial.print(filt[i]);    
    if (i!=NUM_SENSORS-1)
      Serial.print(";");    
    */
    
    // prints the percentage in a range from 22% to 100% as ASCII chars from 48 ('0') to 126 ('~')
    float t = (float)filt[i]/(float)base[i]; 
    t *= 100.0; 
    if (t>100) 
      t=100; 
    if (t<22)
      t = 22;
    byte b = (byte)t;
    char c = b+26;// capped at '`'
    Serial.print(c); 
    }
  Serial.print(DATA_RAW_END);       
  }


void sendBinaryData()
  {
  // byte array for storing sensor data  
  byte bytes[NUM_BYTES]; 

  // initialise to zero values
  for (int i=0;i<NUM_BYTES;i++) 
    bytes[i]=0;        
      
  // fill with sensor data
  for (int i=0;i<NUM_SENSORS;i++)
    if (sensorState[i]) 
      bytes[i/8] |= (1 << (myMod(i, 8)));

  // send down the serial port
  Serial.print(DATA_START);   
  for (int i=0;i<NUM_BYTES-1;i++) 
    {
    Serial.print((int)bytes[i]);
    Serial.print(',');
    }
  Serial.print((int)bytes[NUM_BYTES-1]);  
  Serial.print(DATA_END);   
  }


// read the baseline vals
void read_MPR121_baseline(int boardIndex)
  {
  byte devRead  = getRead(boardIndex);
  byte devWrite = getWrite(boardIndex);
  
  uint16_t baseline;
  
  for (int i=0;i<SENSORS_PER_BOARD;i++)
    {
    baseline = mpr121Read(devRead, devWrite, BASELINE_0+i); 
    baseline = baseline << 2;  
    int ind = mapSensor(boardIndex-1, i);
    if (ind!=-1)
      base[ind] = baseline;  
    }  
  }

// read the raw vals
void read_MPR121_raw(int boardIndex)
  {
  read_MPR121_baseline(boardIndex);
    
  byte devRead  = getRead(boardIndex);
  byte devWrite = getWrite(boardIndex);
  
  uint16_t filter; 
    
  for (int i=0;i<SENSORS_PER_BOARD;i++)
    {
    filter  =  mpr121Read(devRead, devWrite, FILTDATA_0L+(i*2)); 
    filter |= (mpr121Read(devRead, devWrite, FILTDATA_0L+(i*2)+1) << 8);  
    
    int ind = mapSensor(boardIndex-1, i);
    if (ind!=-1)
      filt[ind] = filter;
    }
  }



// read the on/off status
void Read_MPR121(int boardIndex)
  {
  byte devRead  = getRead(boardIndex);
  byte devWrite = getWrite(boardIndex);
  
  unsigned char status1=mpr121Read(devRead, devWrite, 0x00);
  unsigned char status2=mpr121Read(devRead, devWrite, 0x01);

  byte s = 0x01; 
  for (int i=0;i<8;i++)
    {
    int ind = mapSensor(boardIndex-1, i);
    if (ind!=-1)
      sensorState[ind] = status1 & s; 
    s*=0x02; 
    }
  
  s = 0x01; 
  for (int i=8;i<12;i++)
    {
    int ind = mapSensor(boardIndex-1, i);
    if (ind!=-1)
      sensorState[ind] = status2 & s;   
    s*=0x02; 
    }    
 
  }


int mapSensor(int boardIndex, int sensorIndex)
  {
    if (useSolo)
      return mapSensorSolo(sensorIndex);
    
    if (useLarge == false)
      return mapSensor3mm(boardIndex, sensorIndex);
    else
      return mapSensor45mm(boardIndex, sensorIndex);
  }


int mapSensorSolo(int sensorIndex)
  { 
    // this is where we map the actual sensor arrangement onto the device grid structure. 
    // note we are returning 0-47 (to fit into bytes), so figures after midpoint (24) are
    // one less than they should be - the sensor grid has no pin at the mid point. 

       switch (sensorIndex)
         {
         case 0 : return 10;//
         case 1 : return 11;//
         case 2 : return 7;//
         case 3 : return 3;//
         case 4 : return 6;//
         case 5 : return 2;//
         case 6 : return 5;//
         case 7 : return 1;//
         case 8 : return 9;//
         case 9 : return 0;//
         case 10: return 4;//
         case 11: return 8;//
         }
  }



int mapSensor3mm(int boardIndex, int sensorIndex)
  { 
    // this is where we map the actual sensor arrangement onto the device grid structure. 
    // note we are returning 0-47 (to fit into bytes), so figures after midpoint (24) are
    // one less than they should be - the sensor grid has no pin at the mid point. 
    
   switch (boardIndex)
     {
     case 0 : //tl  //grd
       switch (sensorIndex)
         {
         case 0 : return 44;
         case 1 : return 43;
         case 2 : return 42;
         case 3 : return 41;
         case 4 : return 34;
         case 5 : return 27;
         case 6 : return 21;
         case 7 : return 14;
         case 8 : return 7;
         case 9 : return 0;
         case 10: return 1;
         case 11: return 2;
         }
       break;  
       
     
     case 1 :  //tr?  //pwr
       switch (sensorIndex)
         {
         case 0 : return 45;
         case 1 : return 46;
         case 2 : return 47;
         case 3 : return 40;
         case 4 : return 33;
         case 5 : return 26;
         case 6 : return 20;
         case 7 : return 13;
         case 8 : return 6;
         case 9 : return 5;
         case 10: return 4;
         case 11: return 3;
         }
       break;  

     // WRONG - REDO for 3mm board!!!!
     case 2 :  //bl?  - sda
       switch (sensorIndex)
         {
         case 0 : return 36;//-
         case 1 : return 35;//-
         case 2 : return 28;//-
         case 3 : return 22;//-
         
         case 4 : return 29;//- 
         case 5 : return 30;//-
         case 6 : return 16;//-
         case 7 : return 23;//-
         
         case 8 : return 15;//-
         case 9 : return 8;//-
         case 10: return 9;//-
         case 11: return 10;//-
         }
       break;  
      
     case 3 :  //br? // scl
       switch (sensorIndex)
         {
         case 0 : return 37;//-
         case 1 : return 38;//-
         case 2 : return 39;//-
         case 3 : return 32;//-
         
         case 4 : return 24;//-
         case 5 : return 31;//-
         case 6 : return 17;//-
         case 7 : return 18;//-
         
         case 8 : return 25;//-
         case 9 : return 19;//-
         case 10: return 12;//-
         case 11: return 11;//-
         }
       break;    
       
     }
     
     return -1;
    
  }
  
  
// this function for the larger board with 4.5mm interpin spacing
int mapSensor45mm(int boardIndex, int sensorIndex)
  { 
    // this is where we map the actual sensor arrangement onto the device grid structure. 
    // note we are returning 0-47 (to fit into bytes), so figures after midpoint (24) are
    // one less than they should be - the sensor grid has no pin at the mid point. 
    
   switch (boardIndex)
     {
     case 0 : //tl  
       switch (sensorIndex)
         {
         case 0 : return 43;
         case 1 : return 41;
         case 2 : return 42;
         case 3 : return 34;
         case 4 : return 27;//
         case 5 : return 21;//
         case 6 : return 3;//
         case 7 : return 14;
         case 8 : return 2;//
         case 9 : return 7;//
         case 10: return 1;//
         case 11: return 0;//
         }
       break;  
       
     
     case 1 :  //tr?  
       switch (sensorIndex)
         {
         case 0 : return 44;//
         case 1 : return 47;//
         case 2 : return 46;//
         case 3 : return 40;//
         case 4 : return 45;//
         case 5 : return 33;//
         case 6 : return 26;//
         case 7 : return 20;//
         case 8 : return 4;//
         case 9 : return 13;//
         case 10: return 5;//
         case 11: return 6;//
         }
       break;  

      
     case 2 :  //bl?  - sda
       switch (sensorIndex)
         {
         case 0 : return 36;//
         case 1 : return 35;//
         case 2 : return 29;//
         case 3 : return 28;//
         
         case 4 : return 23;// 
         case 5 : return 22;//
         case 6 : return 16;//
         case 7 : return 15;//
         
         case 8 : return 9;//
         case 9 : return 8;//
         case 10: return 17;//
         case 11: return 10;//
         }
       break;  
      
     case 3 :  //br? // scl
       switch (sensorIndex)
         {
         case 0 : return 37;//
         case 1 : return 30;//
         case 2 : return 39;//
         case 3 : return 38;//
         
         case 4 : return 32;//
         case 5 : return 31;//
         case 6 : return 25;//
         case 7 : return 24;//
         
         case 8 : return 19;//
         case 9 : return 18;//
         case 10: return 12;//
         case 11: return 11;//
         }
       break;      
       
     }
     
     return -1;
    
  }  


byte dumbMap(int boardIndex, int sensorIndex)
  {
    return (boardIndex*SENSORS_PER_BOARD)+sensorIndex; 
  }


byte mpr121Read(byte devRead, byte devWrite, uint8_t address)
  {
  byte data;
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte(devWrite);  // write byte
  i2cWaitForComplete();

  i2cSendByte(address);   // write register address
  i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte(devRead);  // read byte
  i2cWaitForComplete();
  i2cReceiveByte(TRUE);
  i2cWaitForComplete();

  data = i2cGetReceivedByte();  // Get MSB result
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);  // Disable TWI
  sbi(TWCR, TWEN);  // Enable TWI

  return data;
  }

void mpr121Write(byte devRead, byte devWrite, unsigned char address, unsigned char data)
  {
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte(devWrite);  // write byte
  i2cWaitForComplete();

  i2cSendByte(address);  // write register address
  i2cWaitForComplete();

  i2cSendByte(data);
  i2cWaitForComplete();

  i2cSendStop();
  }


void mpr121QuickConfig(byte devRead, byte devWrite)
{
  Serial.println("Configuring MPR121.");
  
  // do a soft reset - this reinits the sensor, means we don't have to power off to reset electrode calibration, etc. 
  // may take some time, remove if slow.
   mpr121Write(devRead, devWrite, SOFTRESET, 0x63);
  
  // Section A
  // This group controls filtering when data is > baseline.
  mpr121Write(devRead, devWrite, MHD_R, 0x01);
  
//  mpr121Write(devRead, devWrite, NHD_R, 0x0E); // ADA
  mpr121Write(devRead, devWrite, NHD_R, 0x01); // ORG 0x01, ADAFRUIT 0x0E - means number of sample required to change the baseline (e.g. to treat some observed variation as non-noise)
  // Adafruit version is slower to change baseline - probably not an issue.
  
  mpr121Write(devRead, devWrite, NCL_R, 0x00); // 
  mpr121Write(devRead, devWrite, FDL_R, 0x00);

  // Section B
  // This group controls filtering when data is < baseline.
  mpr121Write(devRead, devWrite, MHD_F, 0x01);
  
//  mpr121Write(devRead, devWrite, NHD_F, 0x05); //ADA
  mpr121Write(devRead, devWrite, NHD_F, 0x01); // ORG 0x01, ADAFRUIT 0x05 - this is the rate at which changes occur (e.g. the step size?)
  // ADAFRUIT NCL_F set to 0x01 - means number of sample required to change the baseline (e.g. to treat some observed variation as non-noise)
  // Adafruit version is faster to change here - probably not an issue.
  
//  mpr121Write(devRead, devWrite, NCL_F, 0x01); // ADA
  mpr121Write(devRead, devWrite, NCL_F, 0xFF); // ORG 0xFF, ADAFRUIT 0x01 
  
//  mpr121Write(devRead, devWrite, FDL_F, 0x00); // ADA
  mpr121Write(devRead, devWrite, FDL_F, 0x02); // ORG 0x02, ADAFRUIT 0x00 // filter speed - higher is slower...
  
  // we miss a section here for filtering when "touched" (see adafruit example - but I think its all set to defaults, so no probs
  
  // Section C
  // This group sets touch and release thresholds for each electrode
  mpr121Write(devRead, devWrite, ELE0_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE0_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE1_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE1_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE2_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE2_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE3_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE3_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE4_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE4_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE5_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE5_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE6_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE6_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE7_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE7_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE8_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE8_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE9_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE9_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE10_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE10_R, REL_THRESH);
  
  mpr121Write(devRead, devWrite, ELE11_T, TOU_THRESH);
  mpr121Write(devRead, devWrite, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  //mpr121Write(FIL_CFG, 0x24); // default value - relates to filter settings on charge time
  
  // auto config?
  mpr121Write(devRead, devWrite, ATO_CFGU, 0xC9);   // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V // 201 in decimal
  mpr121Write(devRead, devWrite, ATO_CFGL, 0x82);   // LSL = 0.65*USL = 0x82 @3.3V          // 131 in decimal
  mpr121Write(devRead, devWrite, ATO_CFGT, 0xB5);   // Target = 0.9*USL = 0xB5 @3.3V        // changed to 200
  mpr121Write(devRead, devWrite, ATO_CFG0, 0x1B);
  
  mpr121Write(devRead, devWrite, FIL_CFG, 0x22); // (0b 0010 0000 / 0x20) - 4ms?  (0b 0010 0010 / 0x22) - 16ms?
  int t =  mpr121Read(devRead, devWrite, FIL_CFG);
  if (t!=0x22)
    Serial.println("************CONFIGURED TOO SLOW***************\nCheck your settings");
  
  // Section E
  // Electrode Configuration
  // Enable 6 Electrodes and set to run mode
  // Set ELE_CFG to 0x00 to return to standby mode
  //mpr121Write(ELE_CFG, 0x0C);	// Enables all 12 Electrodes
  //mpr121Write(ELE_CFG, 0x06);	// Enable first 6 electrodes
  mpr121Write(devRead, devWrite, ELE_CFG, 0x8c); //??


  // Section F
  // Enable Auto Config and auto Reconfig
  //mpr121Write(ATO_CFG0, 0x0B);
  //mpr121Write(ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   mpr121Write(ATO_CFGL, 0x82);	// LSL = 0.65*USL = 0x82 @3.3V
  //mpr121Write(ATO_CFGT, 0xB5);  // Target = 0.9*USL = 0xB5 @3.3V
   
   Serial.println("MPR121 configuration complete.");
}


void printSoloGrid()
  {
  Serial.println(); 
  Serial.println(); 
  for (int i=0;i<NUM_SENSORS;i++)
    {
    if (i%4==0)
      Serial.println(); 
    if (sensorState[i]) 
      Serial.print(1);
    else 
      Serial.print(0);  
    }  
  }

void printBinaryGrid()
  {
  if (useSolo)
    return printSoloGrid();
    
  Serial.println(); 
  Serial.println(); 
  for (int i=0;i<NUM_SENSORS;i++)
    {
    int t = i;
    if (i==24)
      Serial.print("X"); 
    if (i>23)
      t++;
    if (t%7==0)
      Serial.println(); 
      
    if (sensorState[i]) 
      Serial.print(1);
    else 
      Serial.print(0); 
    }
  Serial.println();   
  Serial.println(); 
  }

void processMotor()
  {
  // read the input on analog pin 0:
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A1);
  // print out the value you read:
  Serial.print  (sensorValue1);
  Serial.print  (" ");
  Serial.println(sensorValue2);
  delay(10);        // delay in between reads for stability

  int d = sensorValue1-sensorValue2;
  if (abs(d)>10)
    motor1.drive(-d);
  else
    motor1.drive(0);
  }


