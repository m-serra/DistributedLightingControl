#include <Wire.h>
#include <stdio.h>
#define LDR A0
#define LED 3
#define MEAN_ACQ 5
#define euler 2.718281828

double f = 1.0;
float K11 = 0.0;
float K12 = 0.0;
double K2[24];
float my_o = 0.0;
float other_o = 0.0;
float other_Kii = 0.0;
float other_Kij = 0.0;
char charVal[10];
char charValK11[10];
char charValK12[10];
char charValo[10];
double KK[25];
int p = 0;
double vled;
double Vread = 0.0;
double L = 0.0;
double vi = 0.0;
double Li = 0.0;
double m = -0.7343; // -0.6789; //-0.5421;
double b = 0.7052; //1.86; // 1.4390;
const int address = 0x48; //other device address
double t_start_cycle = 0.0;
double Ts = 0.01;
char init_flag ='0';
char calib_flag1 = '0';
char calib_flag2 = '0';
int a = 1;
char arduino ='0';
char other_arduino ='0';
char end_calib = '0';
char myConcatenation[20];
boolean get_external_luminance = true;
char msg_received[50];
boolean sent_consensus_parameters = false;
boolean received_consensus_parameters = false;
boolean occupancy_state = false;
//const byte interruptPin = 2;
String inputString = "";
bool stringComplete = false;

void setup(){
  Serial.begin(1000000);
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), occupancy, LOW);
  pinMode(LED, OUTPUT);
  pinMode(LDR, INPUT);
}

void loop(){
  if ((micros() - t_start_cycle)*pow(10,-6) >= Ts){
    //Store the time at which the cycle begins
    t_start_cycle = micros();
    CALI();
    if(sent_consensus_parameters && received_consensus_parameters && end_calib == '1'){
      if(arduino == '1'){
        Serial.print("K11:  ");
        Serial.print(K11,4);
        Serial.print("  K12:  ");
        Serial.print(K12,4);
        Serial.print("  K21:  ");
        Serial.print(other_Kij,4);
        Serial.print("  K22:  ");
        Serial.print(other_Kii,4);
        Serial.print("  o1:  ");
        Serial.print(my_o,4);
        Serial.print("  o2:  ");
        Serial.println(other_o,4);
      }
      else if(arduino == '2'){
        Serial.print("K11:  ");
        Serial.print(other_Kii,4);
        Serial.print("  K12:  ");
        Serial.print(other_Kij,4);
        Serial.print("  K21:  ");
        Serial.print(K12,4);
        Serial.print("  K22:  ");
        Serial.print(K11,4);
        Serial.print("  o1:  ");
        Serial.print(other_o,4);
        Serial.print("  o2:  ");
        Serial.println(my_o,4);
      }
      end_calib = '2';
    }

    if(stringComplete){
      Serial.println("recebi uma sring");
      if (inputString == "o\n"){
        //occupied
        
        occupancy_state = true;
      }else if(inputString == "e\n"){
        //empty

        occupancy_state = false;
      }
      stringComplete = false;
      inputString = "";
    }
  }
  
}

void CALI(){
  
  while (end_calib == '0'){ 
    
    if(Wire.available() == 0 && a == 1){ //if nothing is on the bus we can send
      Wire.beginTransmission(address); //get BUS
      Wire.write("1");  
      Wire.endTransmission(); //release BUS
    }
    
    delay(100);
    a = 2;

    if (init_flag == '1'){
      arduino = '2';
      other_arduino = '1';
      Serial.print("Arduino");
      Serial.println(arduino);
      init_flag = '2';
      
    }else if(init_flag == '0'){
      arduino = '1';
      other_arduino = '2';
      Serial.print("Arduino");
      Serial.println(arduino);
      init_flag = '2';
      
    }

    if (arduino == '1' && calib_flag1 == '0'){
      LDR_calib(1);
      Serial.println("eu sou o 1 e eu acabei a minha calibração");
      calib_flag1 = '1';
    }
    
    if (arduino == '2' && calib_flag1 == '1' && calib_flag2 == '0'){
      LDR_calib(2);
      
        if(Wire.available()==0){ //if nothing was on the bus we can send
          Wire.beginTransmission(address); //get BUS
          Wire.write("DD");  
          Wire.endTransmission(); //release BUS
          Serial.println("eu sou o 2 e eu acabei a minha calibração");
          calib_flag2 = '1';
         
        }
      
    }

    if (calib_flag1 == '1' && calib_flag2 == '1'){
      send_parameters();
      end_calib = '1';
    }
  }
}

void receiveEvent(int howMany){
  int i = 0;

  while(Wire.available()> 0){
    char c = Wire.read();

    if(i < 50){
      msg_received[i] = c;
    }
      
    i++;
  }
  
  Serial.print("Received  ");
  Serial.println(msg_received);
    
  if(msg_received[0] == 'L'){    
    sscanf(msg_received, "L=%lf;", &f);
    
    vi = ((analogRead(LDR)*5.0)/1023);
    Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
    
    K2[p] = (Li/f);

    p = p+1;
    
  }

  //done (the other)
  if(msg_received[0] == 'D' && msg_received[1] == 'D'){
    Serial.println("o outro acabou");
    calib_flag2 = '1';
    
  }else if(msg_received[0] == 'D'){ //done (me)
      for (i = 0; i < 24; i++){
        K12 = K12 + K2[i];
      }
  
    K12 = K12/24;
    //Serial.print("K12  ");
    //Serial.println(K12);
    calib_flag1 = '1';
  }

  if(msg_received[0] == 'P'){

    String pch;
    int k = 1;
    
    pch = strtok (msg_received,"P ;");
    while (pch != NULL)
    {
      if (k == 1){ other_Kii = pch.toFloat(); }
      if (k == 2){ other_Kij = pch.toFloat(); }
      if (k == 3){ other_o = pch.toFloat(); }
      
      pch = strtok (NULL, "P ;");

      k++;
    }

    end_calib = '1';
    received_consensus_parameters = true;
  }
  
  if(a == 1){
    init_flag = msg_received[0];
    a = 2;
  } 
      
}


void LDR_calib(char arduino){
  
  int j = 0;
  int i = 0;
  
  for (j = 0; j < 10; j++){
    i = j;
    
    //apply a 10 increment over the LED actuation 
    analogWrite(LED, 28.33*j); //10, 20, 30, 40, 50,..., 250. maximum is 255 

    if(j == 0 && get_external_luminance){
      vi = ((analogRead(LDR)*5.0)/1023);
      my_o = pow(10,-b/m)*pow(((5/vi)-1),1/m);

      Serial.print("my_o:  ");
      Serial.println(my_o);

      get_external_luminance = false;

    }

    if (j != 0){
      vled = ((10.0*j)/255.0)*5.0;
      L = pow(10,-b/m)*pow(((5/vled)-1),1/m);
      
      dtostrf(L, 4, 4, charVal);
      
      //delay
      delay(250);
      sprintf(myConcatenation,"L=%s",charVal);
      
      vi = ((analogRead(LDR)*5.0)/1023);
      Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
      
      if(Wire.available() == 0){ //if nothing is on the bus we can send
        Wire.beginTransmission(address); //get BUS
        Wire.write(myConcatenation);  
        Wire.endTransmission(); //release BUS
      }
      
      KK[i] = Li/L;
      delay(500);
      
    }
  }
  
  analogWrite(LED, 0);

  if(Wire.available()==0){ //if nothing was on the bus we can send
    Wire.beginTransmission(address); //get BUS
    Wire.write("D");  //done
    Wire.endTransmission(); //release BUS
  }
  
  for (i = 1; i < 10; i++){
    K11 = K11 + KK[i];
  }
  
  K11 = K11/10;
    
  calib_flag1 = '1';

}


void send_parameters(){

  Serial.println(my_o);
  Serial.println(other_o);

  float K11_aux = K11;
  float K12_aux = K12;
  float my_o_aux = my_o;
  
  //After calibrate, send a message to the other arduino with our parameters Kii, Kij and oi    
  dtostrf(K11_aux, 4, 4, charValK11);
  dtostrf(K12_aux, 4, 4, charValK12);
  dtostrf(my_o_aux, 4, 4, charValo);
  
  sprintf(myConcatenation,"P %s %s %s;",charValK11, charValK12, charValo);

  //sprintf(myConcatenation,"P %f %f %f;", K11, K12, my_o);
  Serial.print("sending message: ");
  Serial.println(myConcatenation);

  Serial.println(my_o);
  Serial.println(other_o);

  if(Wire.available() == 0){ //if nothing is on the bus we can send
    Wire.beginTransmission(address); //get BUS
    Wire.write(myConcatenation);  
    Wire.endTransmission(); //release BUS
    
  }

  sent_consensus_parameters = true;
}

/*
void occupancy(){
  occupancy_state = !occupancy_state;

  if(occupancy_state){Serial.println("occupied!");}
  else{Serial.println("free!");}

 
   identify which arduino am I
   if occuppancy_state = false: send message to the other arduino saying my occupancy level is '0' which corresponds to an L of... 30LUX?
   if occupancy_state = true: send message to the other arduino saying that my occupancy level if '1' which corresponds to an L of...70LUX?
  
   
}*/

void serialEvent(){

    while(Serial.available()){
        
        char inChar = (char)Serial.read();
        
        inputString += inChar;
      
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}
