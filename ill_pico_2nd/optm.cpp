#include "optm.h"
#include "init.h"

optimizer::optimizer(float cost_ , float Q_, int n_consts_, void (*constraint_fnc_)(float&, const float*, const float*, const float, const int))
  : cost{cost_}, Q{Q_}, n_consts{n_consts_}, constraint_fnc{constraint_fnc_}
{
  A = new float[n_consts * total_nodes];
  for(int i = 0; i < n_consts; i++){
    for(int j = 0; j < total_nodes; j++){
      A[i * total_nodes + j] = 0;
    }
  }
  
  lbd = new float[total_nodes];
  for (int i = 0; i < total_nodes; ++i)
    lbd[i] = 0;

  u = new float[total_nodes];
  for (int i = 0; i < total_nodes; ++i)
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
    for(int j = 0; j < total_nodes; j++){
      A[i * total_nodes + j] = A_[i * total_nodes + j];
    }
  }
  b = b_;
}

void optimizer::update_lbd(float new_lbd, int id){
  lbd[id] = new_lbd;
}

void optimizer::update_u(float new_u, int id){
  u[id] = new_u;
}

void optimizer::iterate_primal(void){
  static float u_new{0};
  u_new = -cost;
  for(int i = 0; i < n_consts; i++){
    u_new += lbd[i]*(A[i * total_nodes + my_id]); 
  }
  u_new /= Q;
  if(u_new < 0) u_new = 0;
  if(u_new > 1) u_new = 1;
  u[my_id] = (u[my_id] + u_new)/2;
}

void optimizer::new_ascent_gain(const float h, const float h_prev){
  static float a_{1.2};
  static float b_{0.8};
  if(h*prev_h>0) ascent_gain *= a_;
  else ascent_gain *= b_;
}

bool optimizer::iterate_dual(void){
  constraint_fnc(h, u, A, b, my_id);
  new_ascent_gain(h, prev_h);
  if(h < threshold) return false;
  
  lbd[my_id] += ascent_gain*h;
  if(lbd[my_id] < 0) lbd[my_id] = 0;
  return true;
}
    