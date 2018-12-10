#include <Wire.h>
#include <stdio.h>
#define LDR A0
#define LED 3
#define MEAN_ACQ 5
#define euler 2.718281828

//Consensus variables
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

node node1;
node node2;
double cost1;
double cost2;
const int L1 = 10; //10LUX is the reference lower bound for our empty desks
const int L2 = 10;
volatile float o1;
volatile float o2;
const int c1 = 1;
const int c2 = 1;
const int c[2] = {c1, c2};
volatile float o[2] = {0.0,0.0};
volatile float K[2][2] = {{0.0,0.0},{0.0,0.0}};
const double rho = 0.07;
volatile double d1[2] = {0,0};
volatile double d2[2] = {0,0};
volatile int consensus_iter = 2;
volatile boolean node_updated = false;
volatile double optimal_d1 = 0.0;
volatile double optimal_d2 = 0.0;
volatile int consensus_iter_aux = 0;


//Calibration variables
volatile double f = 1.0;
volatile float my_Kii = 0.0;
volatile float my_Kij = 0.0;
volatile float my_o = 0.0;
volatile float other_Kii = 0.0;
volatile float other_Kij = 0.0;
volatile float other_o = 0.0;
volatile double K2[24];
volatile char charVal[10];
volatile char charValmy_Kii[10];
volatile char charValmy_Kij[10];
volatile char charVald1[10];
volatile char charVald2[10];
volatile char charValo[10];
volatile char charValiter[10];
volatile double KK[25];
volatile int p = 0;
volatile double vled;
volatile double Vread = 0.0;
volatile double L = 0.0;
volatile double vi = 0.0;
volatile double Li = 0.0;
const double m = -0.7343; // -0.6789; //-0.5421;
const double b = 0.7052; //1.86; // 1.4390;
const int address = 0x48; //broadcast
volatile double t_start_cycle = 0.0;
const double Ts = 0.01;
volatile char init_flag ='0';
volatile char calib_flag1 = '0';
volatile char calib_flag2 = '0';
volatile int a = 1;
volatile char arduino = '0';
volatile char other_arduino = '0';
volatile char end_calib = '0';
char myConcatenation[20];
volatile boolean get_external_luminance = true;
char msg_received[50];
volatile boolean sent_consensus_parameters = false;
volatile boolean received_consensus_parameters = false;
volatile boolean occupancy_state = false;
//const byte interruptPin = 2;
volatile String inputString = "";
volatile bool stringComplete = false;

volatile boolean first = true;

void setup(){
  Serial.begin(1000000);
  
  Wire.begin(address); //join as master
  Wire.onReceive(receiveEvent);
  
  pinMode(LED, OUTPUT);
  pinMode(LDR, INPUT);
}

void loop(){
  if ((micros() - t_start_cycle)*pow(10,-6) >= Ts){
    
    //Store the time at which the cycle begins
    t_start_cycle = micros();

    if(first){
      //Do the initial calibration
      CALI();
  
      if(sent_consensus_parameters && received_consensus_parameters){
        //Initialize consensus
        consensus_initialization();

        //For debug only
        print_node_info(node1, node2);
    
        for(consensus_iter = 0; consensus_iter < 50; consensus_iter++){
          Serial.print("iteration:  ");
          Serial.println(consensus_iter);
          
          if(arduino == '1'){
            if(end_calib == '1'){
              Serial.print("K11:  ");
              Serial.print(my_Kii,4);
              Serial.print("  K12:  ");
              Serial.print(my_Kij,4);
              Serial.print("  K21:  ");
              Serial.print(other_Kij,4);
              Serial.print("  K22:  ");
              Serial.print(other_Kii,4);
              Serial.print("  o1:  ");
              Serial.print(my_o,4);
              Serial.print("  o2:  ");
              Serial.println(other_o,4);
  
              end_calib = '2';
            }
            
            //solve node 1 selfish problem
            cost1 = primal_solve(&node1, rho);
        
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
            dtostrf(node1.d[0], 4, 4, charVald1);
            dtostrf(node1.d[1], 4, 4, charVald2);
    
            sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
          
            Serial.print("sending message: ");
            Serial.println(myConcatenation);
          
            send_message(myConcatenation);
    
            //Wait until I receive proposed solution from arduino 2
            //Update node2 with the d's that arduino 2 sent to me (done in the receiveEvent)
            while (!node_updated){;}
            node_updated = false;
            
            //Compute my average d_av and update my node node1, using the updated node2
            compute_average(&node1, node2);
      
            //Update local lagrangians
            update_lagrangian(&node1, rho);
            
          }else if(arduino == '2'){
            
            if(end_calib == '1'){
              Serial.print("K11:  ");
              Serial.print(other_Kii,4);
              Serial.print("  K12:  ");
              Serial.print(other_Kij,4);
              Serial.print("  K21:  ");
              Serial.print(my_Kij,4);
              Serial.print("  K22:  ");
              Serial.print(my_Kii,4);
              Serial.print("  o1:  ");
              Serial.print(other_o,4);
              Serial.print("  o2:  ");
              Serial.println(my_o,4);
  
              end_calib = '2';
            }
            
            //solve node 2 selfish problem 
            cost2 = primal_solve(&node2, rho);
      
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
            dtostrf(node2.d[0], 4, 4, charVald1);
            dtostrf(node2.d[1], 4, 4, charVald2);
    
            sprintf(myConcatenation,"C %s %s %s;", charValiter, charVald1, charVald2);
          
            Serial.print("sending message: ");
            Serial.println(myConcatenation);
          
            send_message(myConcatenation);
  
           
            //Wait until I receive proposed solution from arduino 1         
            //Update node1 with the d's that arduino 1 sent to me
            while (!node_updated){;}
            node_updated = false;
            
            //Compute my average d_av and update my node node1, using the updated node2
            compute_average(&node2, node1);
      
            //Update local lagrangians
            update_lagrangian(&node2, rho);
          }
        }
        
        first = false;
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
    
        //Send values??? what for...
      }
      
    }
  }
}

void CALI(){
  
  while (end_calib == '0'){ 

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
        my_Kij = my_Kij + K2[i];
      }
  
    my_Kij = my_Kij/24;
    //Serial.print("my_Kij  ");
    //Serial.println(my_Kij);
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

  if(msg_received[0] == 'C'){

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
    a = 2;
  } 
      
}


void LDR_calib(char arduino){
  
  int j = 0;
  int i = 0;
  
  for (j = 0; j < 10; j++){
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
      vled = ((10.0*j)/255.0)*5.0;
      L = pow(10,-b/m)*pow(((5/vled)-1),1/m);
      
      dtostrf(L, 4, 4, charVal);
      
      //delay
      delay(250);
      sprintf(myConcatenation,"L=%s",charVal);
      
      vi = ((analogRead(LDR)*5.0)/1023);
      Li = pow(10,-b/m)*pow(((5/vi)-1),1/m);
      
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

  sent_consensus_parameters = true;
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
            c1, o1, L1};
  
    //Node 2 initialization
    node2 = {2, {0,0}, {0,0}, {0,0}, {other_Kij, other_Kii}, 
            pow(other_Kij,2) + pow(other_Kii,2), pow(other_Kij,2),
            c2, o2, L2};
            
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
            c1, o1, L1};
  
    //Node 2 initialization
    node2 = {2, {0,0}, {0,0}, {0,0}, {my_Kij,my_Kii}, 
            pow(my_Kij,2) + pow(my_Kii,2), pow(my_Kij,2),
            c2, o2, L2};
  }
  
}

void send_message(char* msg){
  while(1){
    if(Wire.available() == 0){ //if nothing was on the bus we can send
      Wire.beginTransmission(address); //get BUS
      Wire.write(msg);  
      Wire.endTransmission(); //release BUS
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
