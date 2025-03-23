#include "pid.h"
#include <Arduino.h>
#include "init.h"

pid::pid(
        float _h,     // Sampling time constant     
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

float pid::feed_backward(float ref, float out, float G,float dist) { //possibly feed in disturbances 'd'
    y[1] = out; 
    P[1] = k * (ref - y[1]);                       // P term
    D[1] = ad * D[0] - bd * (y[1] - y[0]);         // D term (fixed semicolon)
    u[2] = (P[1] - P[0]) + (I[1] - I[0]) + (D[1] - D[0]); // Delta u
    u[1] = u[2] + u[0];                            // u[2] -> delta u, u[1] -> tk, u[0] -> tk-1
    
    // Limit control with optional feed-forward
    u[3] = (u[1] - dist)/G; // + (ref - d) / G;   // Feed-forward (if needed)
    wind_up(u[3]);                                // Handle windup
    return u[3];
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
        es = - u_fb;
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
    tt = h/(ki + 1);
}

sim::sim() 
 
:  h{0}, G{0}, dist{0}, gamma{0.8}, 
  log_R0{0}, Vi{0}, Vf{0}, k{0}, ref{0,0}, compute{false},
  tau_l{0}, tau_c{0}, lux_est{0}  { }

sim::~sim() {}

void sim::param_est(int _ref) {
    ref[1] = _ref;
    if (ref[0] != _ref) { 
        Vi = DAC_RANGE*R1/(R1 + R0*pow(dist,-gamma));
        // Calculate every ref change 
        float r_ldr_final = R0*pow(ref[1],-gamma); //fnal est ldr
        float r_eq = R1*r_ldr_final/(R1 + r_ldr_final); // equivalent resistance 
        Vf = DAC_RANGE*R1/(r_ldr_final + R1); // estimate of final voltage read
        tau_c = 1/(r_eq * CAP1 * pow(10,-6));
        if (Vf > Vi) {
            tau_l = 1000/(20/5); // 20ms div 5, rise time
        }
        else {
            tau_l = 1000/(30/5); // 30ms div 5, fall time/decay
        }
        compute = true; 
        k = 0;
    }
    ref[0] = ref[1];
}   

float sim::sys_sim() { 
    if (compute && (k*h/(tau_c + tau_l) <= 5/(tau_c + tau_l) )) {
        float vout;
        float r_ldr_est;
        // calculate till equilibrium, k*h >= 5*(tau_c + tau_l)
        vout = Vf + (Vi - Vf)*exp((-k*h*(tau_c + tau_l)));
        Serial.print("vout "); Serial.println(vout);
        r_ldr_est = R1*(DAC_RANGE - vout)/vout;
        Serial.print("Rld est "); Serial.println(r_ldr_est);
        //lux_est = pow(r_ldr_est/R0,-1/gamma);
        lux_est = pow(10,(log_R0 - log10(r_ldr_est))/gamma);
        Serial.print("k value"); Serial.println(k);
        k = k + 1;
        return lux_est;
    } 
    else {
        compute = false;
        return lux_est;
    }
}

void sim::init_sim(float _h,float _G, float _dist) { 
    h = _h;
    G = _G;
    dist = _dist;
    log_R0 = log10(float(R0));
    lux_est = dist;
}