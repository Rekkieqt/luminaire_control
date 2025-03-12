#include "pid.h"
#include <Arduino.h>
#define DAC_RANGE 4095
#define LED_PIN 15

pid::pid(
        double _h,     // Sampling time constant     
        float _k,       // Proportional gain
        float _ki,      // Integral gain
        float _kd,      // Derivative gain
        float _tt,      // Anti-windup constant     
        float _b,       // Proportional set point
        float _c,        // Derivative set point   
        int derN)         // Derivative hyper-parameter (8 - 20)
 
:  h{_h}, b{_b}, c{_c}, tt{_tt},
    k{_k}, ki{_ki}, kd{_kd}, N{_N},
    P{0, 0}, I{0, 0}, D{0, 0}, u{0, 0, 0, 0}, y{0, 0}, es{0} {
    pid_params();
}

pid::~pid() {}

double pid::feed_backward(float ref, float out, float G) { //possibly feed in disturbances 'd'
    y[1] = out; 
    P[1] = k * (ref - y[1]);                       // P term
    D[1] = ad * D[0] - bd * (y[1] - y[0]);         // D term (fixed semicolon)
    u[2] = (P[1] - P[0]) + (I[1] - I[0]) + (D[1] - D[0]); // Delta u
    u[1] = u[2] + u[0];                            // u[2] -> delta u, u[1] -> tk, u[0] -> tk-1
    
    // Limit control with optional feed-forward
    u[3] = u[1]; // + (ref - d) / G;               // Feed-forward (if needed)
    wind_up(u[3]);                                // Handle windup
    return u[3]*G;
}

void pid::housekeep(float ref) {
    I[1] = I[0] + bi1 * (ref - y[1]) + es * tt;   // I term, including anti-windup error

    // Term shift and housekeeping
    P[0] = P[1];
    D[0] = D[1];
    I[0] = I[1];
    y[0] = y[1];
    u[0] = u[1];
}

void pid::wind_up(float& u_fb) {
    if (u_fb > DAC_RANGE) {
        es = u_fb - DAC_RANGE;
        u_fb = DAC_RANGE;
    } else if (u_fb < 0) {
        es = u_fb;
        u_fb = 0;
    } else {
        es = 0;
    }
}

void pid::pid_params() {
    // Calculate PID parameters
    bi1 = ki * h;
    ad = kd / (kd + N * h);
    bd = kd * N / (kd + N * h);

    // Anti-windup time constant
    tt = h/(ki + 2);
}
