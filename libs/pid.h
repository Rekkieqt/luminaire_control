#ifndef PID_H
#define PID_H
#include "init.h"

class pid 
{

//inline defs
  private:
    //hyper parameters
    float b; // proportional set point
    float c; // derrivative set point
    float k; // proportional gain
    float kd;// derrivative gain
    float ki;// integral gain
    int N;   // derrivative hyper param 8 - 20
    float h;// sampling time constant
    float tt;// anti-wind up constant, adjustable if needed

    //internal variables
    float ad;
    float bd;
    float bi1;
    float es;
    //float bi2;
    float P[2];
    float I[2];
    float D[2];
    float u[4];
    float y[2];
    float out[2];
  public:

    //constructor
    explicit pid(
        float _h,          // Sampling time constant
        float _k = 1,       // Proportional gain
        float _ki = 0.1,    // Integral gain
        float _kd = 0,      // Derivative gain
        float _tt = 0,      // Anti-windup constant
        float _b = 0.5,     // Proportional set point
        float _c = 0,       // Derivative set point
        int derN = 8);          // Derivative hyperparameter
    
    //destructor
    ~pid();

    float feed_backward(float ref, float out, float G,float dist);
    void housekeep(float ref);
    void pid_params();
    void wind_up(float& u_fb);
    
};

#endif //PID_H

