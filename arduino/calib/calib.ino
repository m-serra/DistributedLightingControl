#include <Wire.h>
#define LDR A0
#define LED 3
#define MEAN_ACQ 5
#define euler 2.718281828

double f = 1.0;
double K11 = 0.0;
double K12 = 0.0;
double K2[24];
char charVal[10];
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
char msg_received[20];
char init_flag ='0';
char calib_flag ='0';
char calib_flag1 = '0';
int a = 1;
char arduino ='0';
char other_arduino ='0';
char myConcatenation[20];

void setup(){
  Serial.begin(1000000);
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
  pinMode(LED, OUTPUT);
  pinMode(LDR, INPUT);
}

void loop(){
  if ((micros() - t_start_cycle)*pow(10,-6) >= Ts){
    Serial.println("entrei!");
    //Store the time at which the cycle begins
    t_start_cycle = micros();
    
    if(Wire.available() == 0 && a == 1){ //if nothing is on the bus we can send
      Serial.println("antes de transmitir");
      Wire.beginTransmission(address); //get BUS
      Wire.write("1");  
      Serial.println("depois de escrever");
      Wire.endTransmission(); //release BUS
      Serial.println("fim de transmitir");
    }
    
    delay(100);
    a = 2;

    if (init_flag =='1'){
      arduino = '2';
      other_arduino = '1';
      Serial.print("Arduino");
      Serial.println(arduino);
    }else{
      arduino = '1';
      other_arduino = '2';
      Serial.print("Arduino");
      Serial.println(arduino);
    }

    if (arduino == '1' && calib_flag1=='0'){
      LDR_calib(1);
    }
    
    if (calib_flag1 == '1'){
      LDR_calib(2);
    }
    
    if (calib_flag=='1'){
      vi = ((analogRead(LDR)*5.0)/1023);
      Li=pow(10,-b/m)*pow(((5/vi)-1),1/m);
      K2[p]=(Li/f);
      Serial.print("  K12 ");
      Serial.println(K2[p]);
      p=p+1;
    }   
  }
}


void receiveEvent(int howMany){
  int i = 0;
  
  Serial.print("Received");
  
  while(Wire.available()> 0){
    char c = Wire.read();

    if(strlen(msg_received) < 20){
      msg_received[i] = c;
    }
      
    i++;
  }
  
  calib_flag = '0';
  
  if(msg_received[0] == 'L'){
    calib_flag = '1';
    
    Serial.print(msg_received);
    
    char men[4];
    men[0] = msg_received[2];
    men[1] = msg_received[3];
    men[2] = msg_received[4];
    men[3] = msg_received[5];
    men[4] = msg_received[6];
    
    f = atof(men); 
    vi = ((analogRead(LDR)*5.0)/1023);
    Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
    Serial.print("  ");
    Serial.print(Li);
    
    K2[p] = (Li/f);
    Serial.print("  K12 ");
    Serial.println(K2[p]);
    p = p+1;
    
  }
  
  if(msg_received[0] == 'D'){ //done
    for (i = 0; i < 24; i++){
      K12 = K12 + K2[i];
    }
  
    K12 = K12/24;
    Serial.print("K12  ");
    Serial.println(K12);
    calib_flag1 = '1';
  }
  
  if(a == 1){
    init_flag = msg_received[0];
    a = 2;
  } 
      
}


void LDR_calib(char arduino){
  
  int j = 0;
  int i = 0;

  Serial.println("entrei calib");
  
  for (j = 1; j < 25; j++){
    i = j;
    
    //apply a 10 increment over the LED actuation 
    analogWrite(LED, 10*j); //10, 20, 30, 40, 50,..., 250. maximum is 255 
    
    vled = ((10.0*j)/255.0)*5.0;
    L = pow(10,-b/m)*pow(((5/vled)-1),1/m);
    
    dtostrf(L, 4, 4, charVal);
    
    //delay
    delay(250);
    sprintf(myConcatenation,"L=%s",charVal);
    Serial.print(myConcatenation);
    
    vi = ((analogRead(LDR)*5.0)/1023);
    Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
    Serial.print("  ");
    Serial.print(Li);
    
    if(Wire.available() == 0){ //if nothing is on the bus we can send
      Wire.beginTransmission(address); //get BUS
      Wire.write(myConcatenation);  
      Wire.endTransmission(); //release BUS
    }
    
    KK[i] = Li/L;
    Serial.print("  K11   ");
    Serial.println(KK[i]); 
    delay(500);

  }
  
  analogWrite(LED, 0);
  
  if(Wire.available()==0){ //if nothing was on the bus we can send
    Wire.beginTransmission(address); //get BUS
    Wire.write("D");  //done
    Wire.endTransmission(); //release BUS
  }
  
  for (i = 0; i < 24; i++){
    K11 = K11 + KK[i];
  }
  
  K11 = K11/25;
  
  Serial.print("K11");
  Serial.println(K11);
  
  calib_flag1 = '2';
  
}
