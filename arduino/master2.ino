//master ino
#include <Wire.h>
const int address = 0x48; //other device address

void setup(){
  Serial.begin(1000000);
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
}

void loop(){
  char c;

  if (Serial.available()>0){
    c = Serial.read();
    Serial.print(c);
    Wire.beginTransmission(address); //get BUS
    Wire.write(c); //send byte to address on BUS
    Wire.endTransmission(); //release BUS
  }

  
}

void receiveEvent(int howMany){
  while(Wire.available()>0){
    char c = Wire.read();
    Serial.write(c);
  }
}
