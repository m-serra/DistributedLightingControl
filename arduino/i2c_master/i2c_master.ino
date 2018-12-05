//master ino
#include <Wire.h>
#include <string.h>
#include <time.h>

const int address = 0x48; //other device address
char c;
double t_start_cycle = 0.0;
double Ts = 0.5;
char msg_to_send[7] = "teste1";
char msg_received[7];
int i = 0;
int randomnumber;
int id = 1;
String s;
int len = 8;
char msg_out[8] = "";
int t_i, i_ref, i_meas;
int cycle = 0;

void setup(){
  Serial.begin(1000000);
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
  srand(time(NULL));
}

void loop(){

    // ============================ MUDEI O TS ======================================
    if ( (micros() - t_start_cycle)*pow(10,-6) >= Ts){
      
      //Store the time at which the cycle begins
      t_start_cycle = micros();

      //CHANGE TO READINGS LDR
      i_meas = rand() % 10;
      i_ref = rand() % 10;
      t_i = rand() % 10;
      
      s =String( String(id) + "_" + String(t_i) + "_" + String(i_meas) + "_" + String(i_ref));
      s.toCharArray(msg_out, len);

      if(Wire.available()==0){ //if nothing was on the bus we can send
        Serial.println("Sending...");
        Wire.beginTransmission(address); //get BUS
        Wire.write(msg_out); //send byte to address on BUS
        Wire.endTransmission(); //release BUS
      }

      //cycle += 1;
    }

    /*if(cycle == 5){
      while(1){;}  
    }*/
    
}

void receiveEvent(int howMany){
  i = 0;
  while(Wire.available()>0){
    char c = Wire.read();
    if(strlen(msg_received) < len){
      msg_received[i] = c;
    }
    i++;
  }
  Serial.println(msg_received);
}
