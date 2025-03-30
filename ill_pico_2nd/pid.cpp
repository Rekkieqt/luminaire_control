#include "init.h"
#include "pid.h"
//#include "state.h"

pid::pid(float h_, float b_h_, float b_l_, float c_, float K_h_, float K_l_, float Ti_, float Tt_, float Td_, float N_)
  :h{h_}, b_h{b_h_}, b_l{b_l_}, c{c_}, K_h{K_h_}, K_l{K_l_}, Ti{Ti_}, Tt{Tt_}, Td{Td_}, N{N_}, I{0}, D{0}, y_old{0}
{
  // TODO: check argument validity
}

float pid::get_reference(void){
  return curr_r;
}

void pid::set_reference(float r_){
  curr_r = r_;
  if (occ_st == OCC) { 
    if(feedback) I += curr_K*(curr_b*curr_r - y_old) - K_h*(b_h*curr_r - y_old);
    curr_K = K_h;
    curr_b = b_h;
    r_h = curr_r;
  }
  else if (occ_st == UNOCC) {
    if(feedback) I += curr_K*(curr_b*curr_r - y_old) - K_l*(b_l*curr_r - y_old);
    curr_K = K_l; 
    curr_b = b_l; 
    r_l = curr_r;
  }
}

bool pid::get_anti_wu_status(void){
  return anti_wind_up;
}
void pid::set_anti_wu_status(bool st){
  anti_wind_up = st;
}

bool pid::get_fb_status(void){
  return feedback;
}
void pid::set_fb_status(bool st){
  feedback = st;
}

float pid::get_disturbance(float u){
  return (u < 1 && u > 0) ? curr_r*(1-curr_K*(curr_b-1)) - I : curr_r + u*(Ti/(curr_K*Tt) - G) - Ti/(curr_K*Tt)*v;
}

void pid::set_system_gain(float G_){
  G = G_;
}

float pid::computePWM(float y, float r){
  float P = curr_K*(curr_b*r-y);
  float ad = Td/(Td + N*h);
  float bd = Td*curr_K*N/(Td + N*h);
  D = ad*D - bd*(y - y_old);
  v = feedback ? (P + I + D)/G : (P+curr_K*y)/G;

  float u{0};
  if (v<0) u = 0;
  else if (v>1) u = 1;
  else u = v;

  if(!anti_wind_up) return u;
  I += h/Tt*(u-v); 

  return u;
}

float pid::computePWM(float y){
  return computePWM(y, curr_r);
}
