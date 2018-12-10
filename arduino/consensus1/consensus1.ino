int k11 = 2;
int k12 = 1;
int k21 = 1;
int k22 = 2;

int K[2][2] = {{k11,k12},{k21,k22}};

int L1 = 150;
int o1 = 30;
int L2 = 80;
int o2 = 0;

int c1 = 1;
int c2 = 1;

int c[2] = {c1, c2};
int L[2] = {L1,L2};
int o[2] = {o1,o2};

double rho = 0.07;

double d1[2] = {0,0};
double d2[2] = {0,0};

int i = 2;
boolean first = true;

struct node{
  int index;
  double d[2];
  double d_av[2];
  double y[2];
  int k[2];
  double n;
  double m;
  int c;
  int o;
  int L; 
};

//Node 1 initialization
node node1 = {1, {0,0}, {0,0}, {0,0}, {k11,k12}, 
              pow(k11,2) + pow(k12,2), pow(k12,2),
              c1, o1, L1};

//Node 2 initialization
node node2 = {2, {0,0}, {0,0}, {0,0}, {k21,k22}, 
              pow(k21,2) + pow(k22,2), pow(k21,2),
              c2, o2, L2};


void setup() {
  Serial.begin(9600);

}

void loop() {
  double cost1;
  double cost2;

  if(first == true){
    //iterations
    for(i = 2; i < 10; i++){
  
      //node 1 selfish problem solution
      cost1 = primal_solve(&node1, rho);
      
      Serial.print(node1.d[0],4);
      Serial.print("  ");
      Serial.print(node1.d[1],4);
      Serial.print("  ");
      Serial.println(cost1);L
  
      //node 2 selfish problem solution
      cost2 = primal_solve(&node2, rho);
      
      Serial.print(node2.d[0],4);
      Serial.print("  ");
      Serial.print(node2.d[1],4);
      Serial.print("  ");
      Serial.println(cost2);
  
      //Compute average with available data
      compute_average(&node1, node2);
      compute_average(&node2, node1);
  
      //Update local lagrangians
      update_lagrangian(&node1, rho);
      update_lagrangian(&node2, rho);

    }
    first = false;
  }

}

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
