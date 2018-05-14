import processing.serial.*;
Serial myPort;
String message = null;
PFont fontA;
int fontSize = 12;

int maxNumberOfLabels = 12;

int rectMargin = 40;
int windowWidth = 600;
int windowHeight = rectMargin + (maxNumberOfLabels +1)*(fontSize*2);
int rectWidth = windowWidth - rectMargin*2;
int rectHeight = windowHeight - rectMargin;
int rectCenter = rectMargin + rectWidth/2;

int origin = rectCenter;
int minValue = -1023;
int maxValue = 1023;

float scale = float(rectWidth)/(maxValue-minValue);
String [] sensorLabels = {"s1","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11","s12"};
int labelCount = maxNumberOfLabels;

void setup(){
  //size(windowWidth, windowHeight);
  surface.setSize(windowWidth,windowHeight);
  short portIndex =1;
  String portName = Serial.list()[portIndex];
  println(" Connecting to -> " + portName);
  myPort = new Serial(this, portName, 57600);
  fontA = createFont("Arial.normal", fontSize);
  textFont(fontA);
  labelCount = sensorLabels.length;
}

void drawGrid(){
  fill(0);
  text(minValue, xPos(minValue),rectMargin-fontSize);
  line(xPos(minValue), rectMargin, xPos(minValue), rectHeight + fontSize);
  text((minValue+maxValue)/2, rectCenter, rectMargin-fontSize);
  line(rectCenter, rectMargin,rectCenter, rectHeight+fontSize);
  text(maxValue, xPos(maxValue), rectMargin-fontSize);
  line(xPos(maxValue),rectMargin, xPos(maxValue),rectHeight+fontSize);
  
  for(int i =0; i<labelCount; i++){
    text(sensorLabels[i],fontSize,yPos(i));
    text(sensorLabels[i],xPos(maxValue) + fontSize, yPos(i));
  }  
}

int yPos(int index){
  return rectMargin + fontSize + (index*fontSize*2);
}

int xPos(int value){
  return origin + int(scale*value);
}

void drawBar(int yIndex, int value){
  rect(origin, yPos(yIndex)-fontSize, value*scale, fontSize);
}

void draw(){
  while(myPort.available() >0 ){
    try{
      message = myPort.readStringUntil(10);
      if(message != null){
         print(message);
         String [] data = message.split(",");
         if( data[0].equals("Labels")){
           labelCount = min(data.length-1, maxNumberOfLabels);
           arrayCopy(data,1, sensorLabels, 0, labelCount);
         }else if(data[0].equals("Data")){
           background(255);
           drawGrid();
           fill(204);
           println(data.length);
           for(int i =1; i<=labelCount && 1<data.length-1;i++){
             drawBar(i-1,Integer.parseInt(data[i]));
           }
         }
      }
    }
    catch(Exception e){
    e.printStackTrace();
    }
  }
}