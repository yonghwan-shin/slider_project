import processing.serial.*;


Serial myPort;

int i = 100;

void setup(){
  size(200, 200);
  short portIndex =1;
  String portName = Serial.list()[portIndex];
  myPort = new Serial(this, portName, 115200);
}

void draw(){
  //background(255);
  //int mX = int(map(mouseX,0,width,0,1023));
  //int mY = int(map(mouseY,0,height,1023,0));
  
  //String temp = "@" + mX +","+mY+ "#";
  
  String temp =  i + "$";
  myPort.write(temp);
  i++;
  delay(100);
  //if(i>1000){
  //  i = 100;
  //}
  
  //delay(1000);
  
  
  if(myPort.available() > 0){
    
    
    
    

    println(myPort.readString());
  }
  
}