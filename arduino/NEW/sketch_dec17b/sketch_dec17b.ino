#include <Wire.h>
#include <stdio.h>
#define LDR A0
#define LED 3
#define MEAN_ACQ 5
#define euler 2.718281828

//Control variables
//from 0 to 1023
double avg_read = 0.0, pi_i = 0.0, pi_p = 0.0, vin = 0.0, vf = 0.0, t = 0.0,
       ti = 0.0, tau = 0.0 /*0.013*/, vt = 0.0, l = 0.0, Lu = 0.0, 
       feedforward_term = 0.0, feedback_term = 0.0, gain = 0.0, e = 0.0,
       pi_Kp = 0.5, pi_Ki = 13.5, pi_K1 = pi_Kp*1 /*b=1*/, Ts = 0.015,
       pi_K2 = pi_Kp*pi_Ki*Ts/2, vldr = 0.0, input = 0.0, i_ant = 0.0,
       e_ant = 0.0, sum = 0.0, K_windup = 1.5, r_ldr = 0.0, lux = 0.0;
int j = 0, r1 = 10; //kOhm


//Exceed variables
volatile boolean E_flag = false, E_flag1 = true;
double o_err = 0.0, other_input = 0.0, other_Lu = 0.0;

//I2C Scanner variables
double prev_time=0.0;
byte error, addr;
volatile int nDevices = 0, Recalibrate = 0, incomingByte;

//Occupancy button variables
volatile boolean occupancy_state = false, occupancy_state_prev = false;
const byte interruptPin = 2;
volatile int buttonState = 0;


//Consensus variables
volatile double t_start_cycle_consensus = 0.0;
const double Ts_consensus = 0.01;

struct node{
  int index;
  double d[2];
  double d_av[2];
  double y[2];
  float k[2];
  double n;
  double m;
  int c;
  float o;
  int L; 
};

node node1, node2;
double cost1, cost2;
volatile int my_L = 10, other_L = 10, consensus_iter = 0,
             consensus_iter_aux = 0; //10LUX is the reference lower bound for our empty desks
volatile float o1, o2;
const int c1 = 1, c2 = 1, c[2] = {c1, c2};
volatile float o[2] = {0.0,0.0}, K[2][2] = {{0.0,0.0},{0.0,0.0}};
const double rho = 0.07;
volatile double d1[2] = {0,0}, d2[2] = {0,0};
volatile boolean node_updated = false , receive_data = false;
volatile double optimal_d1 = 0.0, optimal_d2 = 0.0;


//Calibration variables
volatile double f = 1.0, Vread = 0.0, L = 0.0, vi = 0.0, Li = 0.0, vled;
volatile float my_Kii = 0.0, my_Kij = 0.0, my_o = 0.0, other_Kii = 0.0, 
               other_Kij = 0.0, other_o = 0.0;
volatile char charVal[10], charValmy_Kii[10], charValmy_Kij[10], 
              charVald1[10], charVald2[10], charValo[10], charValiter[10],
              init_flag ='0', calib_flag1 = '0', calib_flag2 = '0',
              arduino = '0', other_arduino = '0', end_calib = '0';
volatile double KK[25], K2[24], t_start_cycle = 0.0;
volatile int p = 0, a = 1;
char myConcatenation[32],  msg_received[32];
const double m = -0.7343; // -0.6789; //-0.5421;
const double b = 0.7052; //1.86; // 1.4390;
//const double Ts = 0.01;
const int address = 0x48; //broadcast

volatile boolean get_external_luminance = true, sent_consensus_parameters = false,
                 received_consensus_parameters = true, first = true;

void setup(){
  Serial.begin(1000000);
  
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
  Wire.setClock(100000L);

  //Occupancy button
  attachInterrupt(0, button_pressed, RISING);
  
  pinMode(LED, OUTPUT);
  pinMode(LDR, INPUT);

  //Do the initial calibration
  CALI();

  //Run consensus to get the initial optimal dimming level 
  consensus_initialization();

}

void loop(){
  
    //Store the time at which the cycle begins
    t_start_cycle = micros();
  
    if(occupancy_state != occupancy_state_prev){
      occupancy_state_prev = occupancy_state;
      send_occupancy_state(&node1, &node2);
    }

    if(consensus_iter == 50){
      E_flag1 = true;
      received_consensus_parameters = false;
      E_flag = false;
      
      consensus_iter = 0;

      //After running consensus for consensus_iter, the optimal d for all arduinos has been found
      if(arduino == '1'){
        optimal_d1 = node1.d[0]; //optimal dimming level for arduino 1, found by arduino 1
        optimal_d2 = node1.d[1]; //optimal dimming level for arduino 2, found by arduino 1
      }else if(arduino == '2'){
        optimal_d1 = node2.d[0]; //optimal dimming level for arduino 1, found by arduino 1
        optimal_d2 = node2.d[1]; //optimal dimming level for arduino 2, found by arduino 1
      }
      
      Serial.print("optimal_d1:  ");
      Serial.print(optimal_d1);
      Serial.print("  optimal_d2:  ");
      Serial.println(optimal_d2);

      if(arduino == '1'){
    
        //Final desired value for the voltage in volts
        l = optimal_d1*node1.k[0]+optimal_d2*node1.k[1]+node1.o;
        Serial.print("CHANGED REFERENCE TO:  ");
        Serial.println(l);
        vf = (5/(nthroot(l/pow(10,-b/m),1/m)+1));
        
        //Initial voltage read by the LDR (before the step to the reference value)
        vin = ((analogRead(LDR)*5.0)/1023);
        
        //Serial.println(vi);
        ti = micros();
    
        // find the optimal system gain and tau for the desired LED voltage 
        if (vf <= 1){
          tau= -0.0210*vf  +  0.0372;}
        else if (vf <= 2){
          tau = -0.0048*vf  +  0.0212;}
        else if (vf <= 3){
           tau = -0.0012*vf  +  0.0142;}
        else if (vf <= 4){
          tau=0.0107;}
        else{
          tau = -0.0037*vf  +  0.0250;}
          
        feedforward_term = vf;
      }else if(arduino == '2'){
      
        //Final desired value for the voltage in volts
        l = optimal_d1*node2.k[0]+optimal_d2*node2.k[1]+node2.o;
        vf = (5/(nthroot(l/pow(10,-b/m),1/m)+1));
        
        //Initial voltage read by the LDR (before the step to the reference value)
        vin = ((analogRead(LDR)*5.0)/1023);
        
        //Serial.println(vi);
        ti = micros();
    
        // find the optimal system gain and tau for the desired LED voltage 
        if (vf <= 1){
          tau= -0.0210*vf  +  0.0372;}
        else if (vf <= 2){
          tau = -0.0048*vf  +  0.0212;}
        else if (vf <= 3){
           tau = -0.0012*vf  +  0.0142;}
        else if (vf <= 4){
          tau=0.0107;}
        else{
          tau = -0.0037*vf  +  0.0250;}
          
        feedforward_term = vf;
      }
    }
    
    if(received_consensus_parameters){
      consensus_1iter(false, &node1, &node2);
    } else {
    
      //CONTROL
      
      //reset the sum
      sum = 0.0;
      
      //acquire many values in a single sampling time
      for (j = 0; j < MEAN_ACQ ; j++) sum = sum + analogRead(LDR);
  
      //Low pass filter (mean) to reduce noise
      avg_read = sum/MEAN_ACQ;
  
      //Convert value read (0-1023) to volts (0-5 V)
      vldr = (avg_read/1023.0)*5.0;
      
      //Current time in micros
      t = micros();
  
      //Desired voltage at current time
      vt = vf -((vf - vin)*pow(euler, -((t-ti)*pow(10,-6))/tau));
  
      //Error between the voltage that we should obtain at current time and the voltage read by the LDR at current time
      e = vt - vldr;
  
      //Deadzone
      e=deadzone(e,-0.02,0.02);
      
      pi_p = pi_K1*vt - pi_Kp*vldr;
      pi_i = i_ant + pi_K2*(e + e_ant);
      
      feedback_term = pi_p+pi_i;
         
      input = feedback_term + feedforward_term;
  
      //Anti windup
      if (input < 0){
        pi_i = pi_i + (K_windup)*(0.0-input);
        input = 0;
      }
      else if(input > 4){
        pi_i = pi_i + (K_windup)*(4-input);
        input = 4;
      }
  
      //Write PWM to LED
      if (input<0.08){
      input = 0;
      }
      analogWrite(LED, ((input)*255)/5);
  
       Lu=pow(10,-b/m)*pow(((5/vldr)-1),1/m);
       
      //Prints
      //Serial.print(input);
      //Serial.print("\t");
      //Serial.print(vldr);
      //Serial.print("\t");
      //Serial.print(o_err);
      //Serial.print("\t");
      //Serial.println(Lu);

      //y_ant = vldr;
      i_ant = pi_i;
      e_ant = e;

      if (arduino == '1' && E_flag1 && !E_flag){
        
        dtostrf(input, 4, 4, charValiter);
        dtostrf(Lu, 4, 4, charVald1);
        sprintf(myConcatenation,"R %s %s;", charValiter, charVald1);
        
        //Serial.print("sending rpi message: ");
        //Serial.println(myConcatenation);
        send_message(myConcatenation);

        while(!receive_data){if(node_updated){break;}}
        receive_data = false;

        my_o = Lu-((pow(10,-b/m)*pow(((5/input)-1),1/m)) + (pow(10,-b/m)*pow(((5/other_input)-1),1/m)));
        o_err = abs(my_o - node1.o);
      }
      else if (arduino == '2' && E_flag1 && !E_flag){

        dtostrf(input, 4, 4, charValiter);
        dtostrf(Lu, 4, 4, charVald1);
        sprintf(myConcatenation,"R %s %s;", charValiter, charVald1);
                
        while(!receive_data){ if(node_updated){break;}}
        receive_data = false;

        //Serial.print("sending rpi message: ");
        //Serial.println(myConcatenation);
        send_message(myConcatenation);

        my_o = Lu-((pow(10,-b/m)*pow(((5/input)-1),1/m)) + (pow(10,-b/m)*pow(((5/other_input)-1),1/m)));
        o_err = abs(my_o - node2.o);
      }

      //Serial.print("Lu  ");
      //Serial.print(Lu);
      //Serial.print("  my_o  ");
      //Serial.print(my_o);
      //Serial.print("  o_err  ");
      //Serial.println(o_err);
      
      if(E_flag && E_flag1){
        
        received_consensus_parameters = true;
        consensus_iter = 0;
        
        if (arduino == '1'){
          
          other_o = (other_Lu)-((pow(10,-b/m)*pow(((5/other_input)-1),1/m))*node2.k[1] + (pow(10,-b/m)*pow(((5/input)-1),1/m))*node2.k[0]);
          node1.o = my_o;
          node2.o = other_o;
          }
        else if (arduino == '2'){
  
          other_o = (other_Lu)-((pow(10,-b/m)*pow(((5/other_input)-1),1/m))*node1.k[0] + (pow(10,-b/m)*pow(((5/input)-1),1/m))*node1.k[1]);
          node2.o = my_o;
          node1.o = other_o;
          }
        E_flag1 = false;
        E_flag = false;
      
      }
      if ((o_err > 100) && E_flag1){
        E_flag = true;
        send_message("E");
      }
      }

    
    if((micros() - t_start_cycle)*pow(10,-6) < Ts){ delay((Ts - (micros() - t_start_cycle)*pow(10,-6))*1000); } //if the cycle doesn't exceed the sampling time, sleep for the rest amount of time
    //else{Serial.println((micros() - t_start_cycle)*pow(10,-6));} //prints the time that the cycle took
    else{Serial.println(Ts-(micros()*pow(10,-6) - t_start_cycle*pow(10,-6)),6);}
}

double nthroot(double number, double n){
  if (n == 0) return NAN;
  if (number > 0) return pow(number, 1.0 / n);
  if (number == 0) return 0;
  if (number < 0 && int(n) == n && (int(n) & 1)) return -pow(-number, 1.0 / n);
  return NAN;
}

double deadzone(double x, double xmin, double xmax)
{
if ( x >= xmax )
return x-xmax;
else if (x <= xmin)
return x + xmin;
else
return 0;
}

void CALI(){
  
  while (end_calib == '0'){
   
    //if(nDevices == 0 && a == 1) INIT();
    
    //if (nDevices > 0){

      if(a == 1){
        send_message("1");
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
        send_message("DD");
        Serial.println("eu sou o 2 e eu acabei a minha calibração");
        calib_flag2 = '1';      
      }
  
      if (calib_flag1 == '1' && calib_flag2 == '1'){
        send_parameters();
        end_calib = '1';
      }
    //}
  }
}

void receiveEvent(int howMany){
  double start_time = micros();
  int i = 0;
  char c = " ";

  while(Wire.available() >  0 && c != ';'){
    c = Wire.read();

    if(i < 32){
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
    
  }else if(msg_received[0] == 'D' && msg_received[1] == 'D'){   //done (the other)
    Serial.println("o outro acabou");
    calib_flag2 = '1';
    
  }else if(msg_received[0] == 'D'){ //done (me)
      for (i = 0; i < 24; i++){
        my_Kij = my_Kij + K2[i];
      }
  
    my_Kij = my_Kij/24;
    //Serial.print("my_Kij  ");
    //Serial.println(my_Kij);
    calib_flag1 = '1';
    
  } else if(msg_received[0] == 'O'){

    //Just to check
    if(msg_received[1] == other_arduino){
      
      if(msg_received[2] == '0'){ //other arduino changed the state from occupied to free
        other_L = 10; //FREE -> 10 lux

        if(other_arduino == '1'){
          //update other node
          node1.L = 10;
          
        }else if(other_arduino == '2'){
          //update other node
          node2.L = 10;
        }
        
      }else if(msg_received[2] == '1'){ //other arduino changed the state from free to occupied
        other_L = 55; //OCCUPIED -> 50 lux

        if(other_arduino == '1'){
          //update other node
          node1.L = 55;
          
        }else if(other_arduino == '2'){
          //update other node
          node2.L = 55;
        }
      } 
    }
    
    received_consensus_parameters = true;
    
  } else if(msg_received[0] == 'P'){

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
    
  } else if (msg_received[0] == 'E'){

    E_flag = true;
    
  } else if (msg_received[0] == 'R'){
    String pch;
    int k = 1;
    
    pch = strtok (msg_received,"R ;");
    while (pch != NULL)
    {
      if (k == 1){ other_input = pch.toFloat(); }
      if (k == 2){ other_Lu = pch.toFloat(); }
      
      pch = strtok (NULL, "R ;");

      k++;
    }
    receive_data = true;
  } else if(msg_received[0] == 'C'){
  
    String pch;
    int k = 1;
    float aux1;
    float aux2;
    
    pch = strtok (msg_received,"C ;");
    while (pch != NULL)
    {
      if(arduino == '1'){ //If i'm arduino 1, I've received the proposed solution from arduino2
        if (k == 1){ consensus_iter_aux = (int)pch.toFloat();}
        if (k == 2){ aux1 = pch.toFloat(); } //Update node2 with the d's that arduino 2 sent to me 
        if (k == 3){ aux2 = pch.toFloat(); }
      }else if(arduino == '2'){ //If i'm arduino 2, I've received the proposed solution from arduino1
        if (k == 1){ consensus_iter_aux = (int)pch.toFloat();}
        if (k == 2){ aux1 = pch.toFloat(); } //Update node1 with the d's that arduino 1 sent to me
        if (k == 3){ aux2 = pch.toFloat(); }
      }
      
      pch = strtok (NULL, "C ;");      
      k++;
    }

    if (consensus_iter_aux == consensus_iter){
      if (arduino == '1'){
        node2.d[0] = aux1;
        node2.d[1] = aux2;
      }else if(arduino == '2'){
        node1.d[0] = aux1;
        node1.d[1] = aux2;
      }

      node_updated = true;
    }
  }
  
  if(a == 1){
    init_flag = msg_received[0];
    //nDevices = 1;
    a = 2;
  } 

  //Serial.print("Elapsed time in receiveEvent ISR:  ");
  //Serial.println(micros()-start_time);
  //delayMicroseconds(1000);    
}


void LDR_calib(char arduino){
  
  int j = 0;
  int i = 0;
  
  for (j = 0; j < 9; j++){
    i = j;
    
    analogWrite(LED, 28.33*j);

    if(j == 0 && get_external_luminance){
      vi = ((analogRead(LDR)*5.0)/1023);
      my_o = pow(10,-b/m)*pow(((5/vi)-1),1/m);

      Serial.print("my_o:  ");
      Serial.println(my_o);

      get_external_luminance = false;

    }

    if (j != 0){
      vled = ((28.33*j)/255.0)*5.0;
      L = pow(10,-b/m)*pow(((5/vled)-1),1/m);
      delay(250);
      //Serial.println(L);      
      vi = ((analogRead(LDR)*5.0)/1023);
      Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
      dtostrf(Li, 4, 4, charVal);
      
      //delay
      //delay(250);
      sprintf(myConcatenation,"L=%s",charVal);
      
     
      send_message(myConcatenation);
      
      KK[i] = Li/L;
      delay(500);
      
    }
  }
  
  analogWrite(LED, 0);

  send_message("D");
  
  for (i = 1; i < 10; i++){
    my_Kii = my_Kii + KK[i];
  }
  
  my_Kii = my_Kii/10;
    
  calib_flag1 = '1';

}


void send_parameters(){
  
  //After calibrate, send a message to the other arduino with our parameters Kii, Kij and oi    
  dtostrf(my_Kii, 4, 4, charValmy_Kii);
  dtostrf(my_Kij, 4, 4, charValmy_Kij);
  dtostrf(my_o, 4, 4, charValo);
  
  sprintf(myConcatenation,"P %s %s %s;",charValmy_Kii, charValmy_Kij, charValo);

  //sprintf(myConcatenation,"P %f %f %f;", my_Kii, my_Kij, my_o);
  Serial.print("sending message: ");
  Serial.println(myConcatenation);

  send_message(myConcatenation);

  //sent_consensus_parameters = true;
}

void consensus_initialization(){
 
  if(arduino == '1'){
    //K = {{my_Kii, my_Kij}, {other_Kij, other_Kii}};
    //o = {my_o, other_o};
    K[0][0] = my_Kii;
    K[0][1] = my_Kij;
    K[1][0] = other_Kij;
    K[1][1] = other_Kii;
    o[0] = my_o;
    o[1] = other_o;
    o1 = my_o;
    o2 = other_o;
  
    //Node 1 initialization
    node1 = {1, {0,0}, {0,0}, {0,0}, {my_Kii, my_Kij}, 
            pow(my_Kii,2) + pow(my_Kij,2), pow(my_Kij,2),
            c1, o1, my_L};
  
    //Node 2 initialization
    node2 = {2, {0,0}, {0,0}, {0,0}, {other_Kij, other_Kii}, 
            pow(other_Kij,2) + pow(other_Kii,2), pow(other_Kij,2),
            c2, o2, other_L};
            
  }else if(arduino == '2'){
    K[0][0] = other_Kii;
    K[0][1] = other_Kij;
    K[1][0] = my_Kij;
    K[1][1] = my_Kii;
    o[0] = other_o;
    o[1] = my_o;
    o1 = other_o;
    o2 = my_o;
  
    //Node 1 initialization
    node1 = {1, {0,0}, {0,0}, {0,0}, {other_Kii, other_Kij}, 
            pow(other_Kii,2) + pow(other_Kij,2), pow(other_Kij,2),
            c1, o1, other_L};
  
    //Node 2 initialization
    node2 = {2, {0,0}, {0,0}, {0,0}, {my_Kij,my_Kii}, 
            pow(my_Kij,2) + pow(my_Kii,2), pow(my_Kij,2),
            c2, o2, my_L};
  }
  
}

void send_message(char* msg){
  while(1){
    if(Wire.available() == 0){ //if nothing was on the bus we can send
      Wire.beginTransmission(address); //get BUS
      Wire.write(msg);  
      //Wire.endTransmission(); //release BUS

      if(Wire.endTransmission() != 0){
        Serial.println("ERROR transmitting last message!");
      }
      break;
    }
  }
}


void print_node_info(node _node1, node _node2){

  Serial.println("Node 1");
  Serial.print("index:  ");
  Serial.print(_node1.index);
  Serial.print("  d:  ");
  Serial.print(_node1.d[0]);
  Serial.print("  ");
  Serial.print(_node1.d[1]);
  Serial.print("  d_av:  ");
  Serial.print(_node1.d_av[0]);
  Serial.print("  ");
  Serial.print(_node1.d_av[1]);
  Serial.print("  y:  ");
  Serial.print(_node1.y[0]);
  Serial.print("  ");
  Serial.print(_node1.y[1]);
  Serial.print("  k:  ");
  Serial.print(_node1.k[0]);
  Serial.print("  ");
  Serial.print(_node1.k[1]);
  Serial.print("  n:  ");
  Serial.print(_node1.n);
  Serial.print("  m:  ");
  Serial.print(_node1.m);
  Serial.print("  c:  ");
  Serial.print(_node1.c);
  Serial.print("  o:  ");
  Serial.print(_node1.o);
  Serial.print("  L:  ");
  Serial.println(_node1.L);

  Serial.println("Node 2");
  Serial.print("index:  ");
  Serial.print(_node2.index);
  Serial.print("  d:  ");
  Serial.print(_node2.d[0]);
  Serial.print("  ");
  Serial.print(_node2.d[1]);
  Serial.print("  d_av:  ");
  Serial.print(_node2.d_av[0]);
  Serial.print("  ");
  Serial.print(_node2.d_av[1]);
  Serial.print("  y:  ");
  Serial.print(_node2.y[0]);
  Serial.print("  ");
  Serial.print(_node2.y[1]);
  Serial.print("  k:  ");
  Serial.print(_node2.k[0]);
  Serial.print("  ");
  Serial.print(_node2.k[1]);
  Serial.print("  n:  ");
  Serial.print(_node2.n);
  Serial.print("  m:  ");
  Serial.print(_node2.m);
  Serial.print("  c:  ");
  Serial.print(_node2.c);
  Serial.print("  o:  ");
  Serial.print(_node2.o);
  Serial.print("  L:  ");
  Serial.println(_node2.L);
 
}



/* --------------------- FUNCTIONS FROM CONSENSUS1-----------------------*/

double primal_solve(node* _node, double rho){
  double d_best[2] = {-1,-1};
  double cost_best = 1000000; //large number
  double sol_unconstrained = 1;
  double cost_unconstrained = 1000000;
  double sol_boundary_linear = 1;
  double cost_boundary_linear = 1000000;
  double sol_boundary_0 = 1;
  double cost_boundary_0 = 1000000;
  double sol_boundary_100 = 1;
  double cost_boundary_100 = 1000000;
  double sol_linear_0 = 1;
  double cost_linear_0 = 1000000;
  double sol_linear_100 = 1;
  double cost_linear_100 = 1000000;
  double z[2]; //vector
  double d_u[2]; //vector
  double d_bl[2];
  double d_b0[2];
  double d_b1[2];
  double d_l0[2];
  double d_l1[2];

  z[0] = (rho*_node->d_av[0]) - _node->y[0];
  z[1] = (rho*_node->d_av[1]) - _node->y[1];
  z[_node->index-1] = z[_node->index-1] - _node->c;
  
  //unconstrained minimum
  d_u[0] = (1.0/rho)*z[0];
  d_u[1] = (1.0/rho)*z[1]; 

  //checks if the optimal solution found agrees with the constraints (if
  //it's inside the feasible region or on the boundaries)
  sol_unconstrained = check_feasibility(*_node, d_u);
  
  if (sol_unconstrained == 1){
    //If so, then the solution is valid and the cost is evaluated (value
    //of the cost function to minimize)
    cost_unconstrained = evaluate_cost(*_node, d_u, rho);
    
    if (cost_unconstrained < cost_best){
       //Updates the cost if it's better than the cost of the previous
       //solution
       _node->d[0] = d_u[0];
       _node->d[1] = d_u[1];
 
       return cost_unconstrained;  //IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
                                   //NO NEED TO COMPUTE THE OTHER
                                   
    }
  }

  //Compute minimum constrained to linear boundary    ILB
  d_bl[0] = (1.0/rho)*z[0] - (_node->k[0]/_node->n)*(_node->o-_node->L+(1.0/rho)*(z[0]*_node->k[0]+z[1]*_node->k[1]));
  d_bl[1] = (1.0/rho)*z[1] - (_node->k[1]/_node->n)*(_node->o-_node->L+(1.0/rho)*(z[0]*_node->k[0]+z[1]*_node->k[1]));
  
  //Check feasibility of minimum constrained to linear boundary
  sol_boundary_linear = check_feasibility(*_node, d_bl);

  //Compute cost and if best store new optimum
  if(sol_boundary_linear == 1){
      cost_boundary_linear = evaluate_cost(*_node, d_bl, rho);
      if (cost_boundary_linear < cost_best){
         d_best[0] = d_bl[0];
         d_best[1] = d_bl[1];
         cost_best = cost_boundary_linear;
      }
  }

  //Compute minimum constrained to 0 boundary DLB
  d_b0[0] = (1.0/rho)*z[0];
  d_b0[1] = (1.0/rho)*z[1];
  d_b0[_node->index-1] = 0;

  //Check feasibility of minimum constrained to 0 boundary
  sol_boundary_0 = check_feasibility(*_node, d_b0);
    
  //Compute cost and if best store new optimum
  if (sol_boundary_0 == 1){ 
      cost_boundary_0 = evaluate_cost(*_node, d_b0, rho);
      if (cost_boundary_0 < cost_best){
         d_best[0] = d_b0[0];
         d_best[1] = d_b0[1];
         cost_best = cost_boundary_0;
      }
  }

  //Compute minimum constrained to 100 boundary DUB
  d_b1[0] = (1.0/rho)*z[0];
  d_b1[1] = (1.0/rho)*z[1];
  d_b1[_node->index-1] = 100;

  //Check feasibility of minimum constrained to 100 boundary
  sol_boundary_100 = check_feasibility(*_node, d_b1);

  //Compute cost and if best store new optimum
  if(sol_boundary_100 == 1){ 
      cost_boundary_100 = evaluate_cost(*_node, d_b1, rho);
      if (cost_boundary_100 < cost_best){
         d_best[0] = d_b1[0];
         d_best[1] = d_b1[1];
         cost_best = cost_boundary_100;
      }
  }

  //Compute minimum constrained to linear and 0 boundary  ILB^DLB
  d_l0[0] = (1.0/rho)*z[0] - (1/_node->m)*_node->k[0]*(_node->o-_node->L) + ((1.0/rho)/(_node->m))*_node->k[0]*(_node->k[_node->index-1]*z[_node->index-1]-(z[0]*_node->k[0]+z[1]*_node->k[1]));
  d_l0[1] = (1.0/rho)*z[1] - (1/_node->m)*_node->k[1]*(_node->o-_node->L) + ((1.0/rho)/(_node->m))*_node->k[1]*(_node->k[_node->index-1]*z[_node->index-1]-(z[0]*_node->k[0]+z[1]*_node->k[1]));
  d_l0[_node->index-1] = 0;
  
  //Check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_0 = check_feasibility(*_node, d_l0);

  //Compute cost and if best store new optimum
  if (sol_linear_0 == 1){ 
      cost_linear_0 = evaluate_cost(*_node, d_l0, rho);
      if (cost_linear_0 < cost_best){
         d_best[0] = d_l0[0];
         d_best[1] = d_l0[1];
         cost_best = cost_linear_0;
      }
  }

  //Compute minimum constrained to linear and 100 boundary  ILB^DUB
  d_l1[0] = (1.0/rho)*z[0] - (1/_node->m)*_node->k[0]*(_node->o-_node->L+100*_node->k[_node->index-1]) + ((1.0/rho)/(_node->m))*_node->k[0]*(_node->k[_node->index-1]*z[_node->index-1]-(z[0]*_node->k[0]+z[1]*_node->k[1]));
  d_l1[1] = (1.0/rho)*z[1] - (1/_node->m)*_node->k[1]*(_node->o-_node->L+100*_node->k[_node->index-1]) + ((1.0/rho)/(_node->m))*_node->k[1]*(_node->k[_node->index-1]*z[_node->index-1]-(z[0]*_node->k[0]+z[1]*_node->k[1]));
  d_l1[_node->index-1] = 100;

  //Check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_100 = check_feasibility(*_node, d_l1);

  //Compute cost and if best store new optimum
  if (sol_linear_100 == 1){ 
      cost_linear_100 = evaluate_cost(*_node, d_l1, rho);
      
      if (cost_linear_100 < cost_best){
         d_best[0] = d_l1[0];
         d_best[1] = d_l1[1];
         cost_best = cost_linear_100;
      }
  }

  _node->d[0] = d_best[0];
  _node->d[1] = d_best[1];
  
  return cost_best;
    
}

void compute_average(node* _node, node _nodee){

  _node->d_av[0] = (_node->d[0]+_nodee.d[0])/2.0;
  _node->d_av[1] = (_node->d[1]+_nodee.d[1])/2.0;

}

void update_lagrangian(node* _node, double rho){
  
  _node->y[0] = _node->y[0] + rho*(_node->d[0] - _node->d_av[0]);
  _node->y[1] = _node->y[1] + rho*(_node->d[1] - _node->d_av[1]);
  
}

int check_feasibility(node _node, double* d){
  
  double tol = 0.001; //tolerance for rounding errors

  if (d[_node.index-1] < 0-tol) return 0;
  if (d[_node.index-1] > 100+tol) return 0;
  if((d[0]*_node.k[0]+d[1]*_node.k[1]) < _node.L - _node.o - tol) return 0;
  
  return 1;
  
}


double evaluate_cost(node _node, double* d, double rho){
  return (_node.c*d[_node.index-1] + (_node.y[0]*(d[0] - _node.d_av[0]) + _node.y[1]*(d[1] - _node.d_av[1])) + (rho/2.0)*(pow(d[0]-_node.d_av[0],2)+pow(d[1]-_node.d_av[1],2)));
}

/********************************************************************
  Function to change occupancy state when the button is pressed,
  it also changes the reference lower bound (L1 or L2, depending if
  it's arduino 1 or 2) and sends a message to the I2C bus to notify
  the other arduino that my L has changed (and also the RPI)
*********************************************************************/

void button_pressed(){
    buttonState = digitalRead(interruptPin);
    
    delayMicroseconds(5000);
    
    if (buttonState == digitalRead(interruptPin)){
      //Change in occupancy state!
      occupancy_state = !occupancy_state;
    }
}

void send_occupancy_state(node* _node, node* _nodee){
  
    if(occupancy_state){
            
      if(arduino == '1'){
        send_message("O11");

        //update my node
        _node->L = 55;
      }
      else if(arduino == '2'){
        send_message("O21");

        //update my node
        _nodee->L = 55;
      }

      my_L = 55; //OCCUPIED -> 55 lux
      
    }
    else{
      
      if(arduino == '1'){
        send_message("O10");
        
        //update my node
        _node->L = 10;
      }
      else if(arduino == '2'){
        send_message("O20");

        //update my node
        _nodee->L = 10;
      }

      my_L = 10; //FREE -> 10 lux
    }

    //sent_consensus_parameters = true;
    received_consensus_parameters = true;
    //run_consensus(false);
}

/********************************************************************
  Function to check how many devices are connected to the I2C network
  It's used to only start the calibration after detecting that the 
  other arduino is connected to the I2C bus.
*********************************************************************/

void INIT(){
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(addr = 1; addr < 127; addr++ )
  {
    // The i2c_scanner uses the return value of the Write.endTransmisstion 
    // to see if a device did acknowledge to the address.
    if(Wire.available() == 0){
      Wire.beginTransmission(addr);
      Wire.write("1");
      error = Wire.endTransmission();
    }
    
    if (error == 0)
    {
      //Print message
      Serial.print("I2C device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println("  !");
 
      nDevices++;
      
    }else if (error == 4)
    {
      //Print message
      Serial.print("Unknown error at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr,HEX);
      
    } 
       
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}



void run_consensus(boolean first, node * _node, node * _nodee){

  //Wait until it receives consensus parameters 
  //while(!received_consensus_parameters && !sent_consensus_parameters){;}
  while(!received_consensus_parameters){;}

  received_consensus_parameters = false;
  //sent_consensus_parameters = false;

//  if (first){
//    //Initialize consensus
//    consensus_initialization();
//  }

  //For debug only
  print_node_info(*_node, *_nodee);

  for(consensus_iter = 0; consensus_iter < 50; consensus_iter++){
    //Store the time at which the cycle begins
    t_start_cycle_consensus = micros();
  
//    Serial.print("iteration:  ");
//    Serial.println(consensus_iter);
    
    if(arduino == '1'){
//      if (consensus_iter == 0){
//        Serial.print("K11:  ");
//        Serial.print(my_Kii,4);
//        Serial.print("  K12:  ");
//        Serial.print(my_Kij,4);
//        Serial.print("  K21:  ");
//        Serial.print(other_Kij,4);
//        Serial.print("  K22:  ");
//        Serial.print(other_Kii,4);
//        Serial.print("  o1:  ");
//        Serial.print(my_o,4);
//        Serial.print("  o2:  ");
//        Serial.println(other_o,4);
//      }
      
      
      //solve node 1 selfish problem
      cost1 = primal_solve(_node, rho);
  
      //print arduino 1 solution for the dimming level d of arduino 1
      //Serial.print(node1.d[0],4);
      //Serial.print("  ");
      //print arduino 1 solution for the dimming level d of arduino 2
      //Serial.print(node1.d[1],4);
      //Serial.print("  ");
      //print the cost of the solution found by arduino 1
      //Serial.println(cost1);

      //Send my proposed solution to arduino 2
      dtostrf(consensus_iter, 4, 4, charValiter);
      dtostrf(_node->d[0], 4, 4, charVald1);
      dtostrf(_node->d[1], 4, 4, charVald2);

      sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
    
      Serial.print("sending message: ");
      Serial.println(myConcatenation);
    
      send_message(myConcatenation);

      //Wait until I receive proposed solution from arduino 2
      //Update node2 with the d's that arduino 2 sent to me (done in the receiveEvent)
      while (!node_updated){;}
      node_updated = false;
      
      //Compute my average d_av and update my node node1, using the updated node2
      compute_average(_node, *_nodee);

      //Update local lagrangians
      update_lagrangian(_node, rho);
      
    }else if(arduino == '2'){
      
//      if (consensus_iter == 0){
//        Serial.print("K11:  ");
//        Serial.print(other_Kii,4);
//        Serial.print("  K12:  ");
//        Serial.print(other_Kij,4);
//        Serial.print("  K21:  ");
//        Serial.print(my_Kij,4);
//        Serial.print("  K22:  ");
//        Serial.print(my_Kii,4);
//        Serial.print("  o1:  ");
//        Serial.print(other_o,4);
//        Serial.print("  o2:  ");
//        Serial.println(my_o,4);
//      }
      
      //solve node 2 selfish problem 
      cost2 = primal_solve(_nodee, rho);

      //print arduino 2 solution for the dimming level d of arduino 1
      //Serial.print(node2.d[0],4);
      //Serial.print("  ");
      //print arduino 2 solution for the dimming level d of arduino 2
      //Serial.print(node2.d[1],4);
      //Serial.print("  ");
      //print the cost of the solution found by arduino 2
      //Serial.println(cost2);

      //Send my proposed solution to arduino 1
      dtostrf(consensus_iter, 4, 4, charValiter);
      dtostrf(_nodee->d[0], 4, 4, charVald1);
      dtostrf(_nodee->d[1], 4, 4, charVald2);

      sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
    
      Serial.print("sending message: ");
      Serial.println(myConcatenation);
    
      send_message(myConcatenation);
     
      //Wait until I receive proposed solution from arduino 1         
      //Update node1 with the d's that arduino 1 sent to me
      while (!node_updated){;}
      node_updated = false;
      
      //Compute my average d_av and update my node node1, using the updated node2
      compute_average(_nodee, *_node);

      //Update local lagrangians
      update_lagrangian(_nodee, rho);
             
    }

    //Serial.print("sending message: ");
    //Serial.println("E");
    //send_message("E");
    //while(!go_to_next_iter){;}
    //go_to_next_iter = false; 
    
    //if((micros() - t_start_cycle_consensus)*pow(10,-6) < Ts_consensus){ delay((Ts_consensus - (micros() - t_start_cycle_consensus)*pow(10,-6))*1000); } //if the cycle doesn't exceed the sampling time, sleep for the rest amount of time
    //else{Serial.println((micros() - t_start_cycle_consensus)*pow(10,-6),5);} //prints the time that the cycle took
  
  }

  //After running consensus for consensus_iter, the optimal d for all arduinos has been found
  if(arduino == '1'){
    optimal_d1 = _node->d[0]; //optimal dimming level for arduino 1, found by arduino 1
    optimal_d2 = _node->d[1]; //optimal dimming level for arduino 2, found by arduino 1
  }else if(arduino == '2'){
    optimal_d1 = _nodee->d[0]; //optimal dimming level for arduino 1, found by arduino 1
    optimal_d2 = _nodee->d[1]; //optimal dimming level for arduino 2, found by arduino 1
  }
  
  Serial.print("optimal_d1:  ");
  Serial.print(optimal_d1);
  Serial.print("  optimal_d2:  ");
  Serial.println(optimal_d2);

  consensus_iter = 0;

}


void consensus_1iter(boolean first, node * _node, node * _nodee){
  
    //Wait until it receives consensus parameters 
    while(!received_consensus_parameters){;}

    if (first){
      //Initialize consensus
      consensus_initialization();
    }
    
    //Store the time at which the cycle begins
    t_start_cycle_consensus = micros();
  
    Serial.print("iteration:  ");
    Serial.println(consensus_iter);
    
    if(arduino == '1'){
      
//      if (consensus_iter == 0){
//        Serial.print("K11:  ");
//        Serial.print(my_Kii,4);
//        Serial.print("  K12:  ");
//        Serial.print(my_Kij,4);
//        Serial.print("  K21:  ");
//        Serial.print(other_Kij,4);
//        Serial.print("  K22:  ");
//        Serial.print(other_Kii,4);
//        Serial.print("  o1:  ");
//        Serial.print(my_o,4);
//        Serial.print("  o2:  ");
//        Serial.println(other_o,4);
//      }
      
      
      //solve node 1 selfish problem
      cost1 = primal_solve(_node, rho);
  
      //print arduino 1 solution for the dimming level d of arduino 1
      //Serial.print(node1.d[0],4);
      //Serial.print("  ");
      //print arduino 1 solution for the dimming level d of arduino 2
      //Serial.print(node1.d[1],4);
      //Serial.print("  ");
      //print the cost of the solution found by arduino 1
      //Serial.println(cost1);

      //Send my proposed solution to arduino 2
      dtostrf(consensus_iter, 4, 4, charValiter);
      dtostrf(_node->d[0], 4, 4, charVald1);
      dtostrf(_node->d[1], 4, 4, charVald2);

      sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
    
      Serial.print("sending message: ");
      Serial.println(myConcatenation);
      
      
      send_message(myConcatenation);


      //Wait until I receive proposed solution from arduino 2
      //Update node2 with the d's that arduino 2 sent to me (done in the receiveEvent)
      while (!node_updated){;}
      node_updated = false;
      
      //Compute my average d_av and update my node node1, using the updated node2
      compute_average(_node, *_nodee);

      //Update local lagrangians
      update_lagrangian(_node, rho);
      
    }else if(arduino == '2'){
      
//      if (consensus_iter == 0){
//        Serial.print("K11:  ");
//        Serial.print(other_Kii,4);
//        Serial.print("  K12:  ");
//        Serial.print(other_Kij,4);
//        Serial.print("  K21:  ");
//        Serial.print(my_Kij,4);
//        Serial.print("  K22:  ");
//        Serial.print(my_Kii,4);
//        Serial.print("  o1:  ");
//        Serial.print(other_o,4);
//        Serial.print("  o2:  ");
//        Serial.println(my_o,4);
//      }
      
      //solve node 2 selfish problem 
      cost2 = primal_solve(_nodee, rho);

      //print arduino 2 solution for the dimming level d of arduino 1
      //Serial.print(node2.d[0],4);
      //Serial.print("  ");
      //print arduino 2 solution for the dimming level d of arduino 2
      //Serial.print(node2.d[1],4);
      //Serial.print("  ");
      //print the cost of the solution found by arduino 2
      //Serial.println(cost2);
      
      //Wait until I receive proposed solution from arduino 1         
      //Update node1 with the d's that arduino 1 sent to me
      while (!node_updated){;}
      node_updated = false;

      //Send my proposed solution to arduino 1
      dtostrf(consensus_iter, 4, 4, charValiter);
      dtostrf(_nodee->d[0], 4, 4, charVald1);
      dtostrf(_nodee->d[1], 4, 4, charVald2);

      sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
    
      Serial.print("sending message: ");
      Serial.println(myConcatenation);
    
      send_message(myConcatenation);
      
      //Compute my average d_av and update my node node1, using the updated node2
      compute_average(_nodee, *_node);

      //Update local lagrangians
      update_lagrangian(_nodee, rho);
             
    }

    consensus_iter++;

    //Serial.print("sending message: ");
    //Serial.println("E");
    //send_message("E");
    //while(!go_to_next_iter){;}
    //go_to_next_iter = false; 
    
    //if((micros() - t_start_cycle_consensus)*pow(10,-6) < Ts_consensus){ delay((Ts_consensus - (micros() - t_start_cycle_consensus)*pow(10,-6))*1000); } //if the cycle doesn't exceed the sampling time, sleep for the rest amount of time
    //else{Serial.println((micros() - t_start_cycle_consensus)*pow(10,-6),5);} //prints the time that the cycle took
}
