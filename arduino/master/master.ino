//master ino
#include <Wire.h>
#include <string.h>

const int address = 0; //other device address
char c;
double t_start_cycle = 0.0;
double Ts = 0.01;
char msg_to_send[7] = "teste1";
char msg_received[7];
int i = 0;

void setup(){
  Serial.begin(1000000);
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
}

void loop(){
  
    if ( (micros() - t_start_cycle)*pow(10,-6) >= Ts){
      
      //Store the time at which the cycle begins
      t_start_cycle = micros();

     
      if(Wire.available()==0){ //if nothing was on the bus we can send
        Serial.println("sending...");
        Wire.beginTransmission(address); //get BUS
        Wire.write(msg_to_send); //send byte to address on BUS
        Wire.endTransmission(); //release BUS
      }
    }
}


void receiveEvent(int howMany){
  i = 0;
  while(Wire.available()>0){
    char c = Wire.read();
    if(strlen(msg_received) < 6){
      msg_received[i] = c;
    }
    i++;
  }
  Serial.println(msg_received);
}
