#include "optm.h"
#include <Arduino.h>


uint8_t total_nodes{static_cast<uint8_t>(maxId + 1)};
uint8_t my_id{myIdentifier};

void constraint(float& h, const float* x, const float*A, const float b, const int my_id){
  float line_sum{0};
  for(uint8_t j = 0; j < total_nodes; j++){
    line_sum += -A[my_id * total_nodes + j]*x[j]; // A = -K
  }
  h = line_sum + b; // b = L-d
}

optimizer::optimizer(float cost_ , float Q_, int n_consts_, void (*constraint_fnc_)(float&, const float*, const float*, const float, const int))
  : cost{cost_}, Q{Q_}, n_consts{n_consts_}, constraint_fnc{constraint_fnc_}
{
  A = new float[n_consts * total_nodes];
  for(int i = 0; i < n_consts; i++){
    for(uint8_t j = 0; j < total_nodes; j++){
      A[i * total_nodes + j] = 0;
    }
  }
  
  lbd = new float[total_nodes];
  for (uint8_t i = 0; i < total_nodes; ++i)
    lbd[i] = 0;

  u = new float[total_nodes];
  for (uint8_t i = 0; i < total_nodes; ++i)
    u[i] = 0;
}

void optimizer::set_cost(float c){
  cost = c;
}

void optimizer::set_threshold(float t){
  threshold = t;
}

void optimizer::set_constraints(float *A_, float b_){
  for(int i = 0; i < n_consts; i++){
    for(uint8_t j = 0; j < total_nodes; j++){
      A[i * total_nodes + j] = A_[i * total_nodes + j];
    }
  }
  b = b_;
}

void optimizer::set_cnstr_fn(int n_consts_ , void (*constraint_fnc_)(float&, const float*, const float*, const float, const int)){
  n_consts = n_consts_;

  total_nodes = static_cast<uint8_t>(maxId + 1);
  my_id = myIdentifier;
  
  delete[] A;
  delete[] lbd;
  delete[] u;

  A = new float[n_consts * total_nodes];
  for(int i = 0; i < n_consts; i++){
    for(uint8_t j = 0; j < total_nodes; j++){
      A[i * total_nodes + j] = 0;
    }
  }
  
  lbd = new float[total_nodes];
  for (uint8_t i = 0; i < total_nodes; ++i)
    lbd[i] = 0;

  u = new float[total_nodes];
  for (uint8_t i = 0; i < total_nodes; ++i)
    u[i] = 0;


  constraint_fnc = constraint_fnc_;
}

void optimizer::set_cnstr_fn(int n_consts_){
  set_cnstr_fn(n_consts_, constraint);
}

float optimizer::get_sol(){
  return u[my_id];
}

void optimizer::update_lbd(float new_lbd, uint8_t id){
  lbd[id] = new_lbd;
}

void optimizer::update_u(float new_u, uint8_t id){
  u[id] = new_u;
}

bool optimizer::iterate_primal(float& u_){
  float u_new{0};
  u_new = -cost;
  for(int i = 0; i < n_consts; i++){
    u_new += lbd[i]*(A[i * total_nodes + my_id]); 
  }
  u_new /= Q;
  if(u_new < 0) u_new = 0;
  if(u_new > 1) u_new = 1;
  u[my_id] = (u[my_id] + u_new)/2;

  if(abs(u[my_id] - u_prev) < threshold && h <= 0) return false;
  u_ = u[my_id];
  u_prev = u[my_id];
  Serial.printf("Novo u (%d) : %.4f\n", my_id, u[my_id]);
  return true;
}


void optimizer::new_ascent_gain(const float h, const float h_prev){
  static float a_{1.2};
  static float b_{0.8};
  if(h*prev_h>0) ascent_gain *= a_;
  else ascent_gain *= b_;
}

float optimizer::iterate_dual(void){
  constraint_fnc(h, u, A, b, my_id);
  new_ascent_gain(h, prev_h);
  Serial.printf("cntr (%d) : %.4f\n", my_id, h);
  
  lbd[my_id] += ascent_gain*h;
  if(lbd[my_id] < 0) lbd[my_id] = 0;
  prev_h = h;
  Serial.printf("Novo lbd (%d) : %.4f\n", my_id, lbd[my_id]);
  return lbd[my_id];
}
    