#ifndef SIM_H
#define SIM_H
#include "init.h"

class sim 
{

//inline defs
  private:
    //hyper parameters
    int k;
    float Vi;
    float Vf;
    float ref[2];
    float gamma;
    float R0_ldr;
    float h; 
    float log_R0; 
    float G; 
    float tau_c;
    float tau_l;
    float lux_est;
    bool compute;
  public:

    //constructor
    explicit sim( );  
    
    //destructor
    ~sim();

    void param_est(float _ref);
    float sys_sim();
    void init_sim(float _h, float _G, float _dist);
    
};
#endif //SIM_H

