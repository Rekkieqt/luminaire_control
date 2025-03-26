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

float pid::feed_backward(float ref, float out, float G, float dist, user_set_flags flag) { //possibly feed in disturbances 'd'
    if (flag.feed_backward) { //flag.feed_backward
        y[1] = out; 
        P[1] = k * (b*ref - y[1]);                     // P term
        D[1] = ad * D[0] - bd * (y[1] - y[0]);         // D term
        u[2] = (P[1] - P[0]) + (I[1] - I[0]) + (D[1] - D[0]); // Delta u
        //Serial.print("I tot "); Serial.println((I[1] - I[0]),3);
        u[1] = u[2] + u[0];                            // u[2] -> delta u, u[1] -> tk, u[0] -> tk-1
        //Serial.print("u[1] "); Serial.println(u[1],3);
        // Feed-forward 
        // Limit control with  feed-forward
        u[3] = (u[1] + ref - dist)/G;  // ufb + uff - dist  
        wind_up(u[3],flag.anti_windup);// Handle windup //flag.anti_windup
    }
    else {
        u[3] = (ref-dist)/G;
    }
    return u[3];
}

void pid::housekeep(float ref) {
    I[0] = I[1];
    I[1] = I[0] + bi1 * (ref - y[1]) + es * h * (1/tt);   // I term, including anti-windup error
    //Serial.print("I1 term "); Serial.println(I[1],4);
    //Serial.print("I0 term "); Serial.println(I[0],4);
    //Serial.print("err "); Serial.println(ref - y[1]);
    //Serial.print("ref "); Serial.println(ref);
    //Serial.print("bi1 "); Serial.println(bi1);
    //Serial.print("Out "); Serial.println(y[1]);
    //Serial.print("es "); Serial.println(es);
    // Term shift and housekeeping
    P[0] = P[1];
    D[0] = D[1];
    y[0] = y[1];
    u[0] = u[1];
}

void pid::wind_up(float& u_fb, volatile bool aa_ctr) {
    if (u_fb > DAC_RANGE && aa_ctr) {
        es = DAC_RANGE - u_fb;
        u_fb = DAC_RANGE;
    } else if (u_fb < 0 && aa_ctr) {
        es =  - u_fb;
        u_fb = 0;
    } else {
        es = 0;
    }
}

void pid::pid_params() {
    // Calculate PID parameters
    bi1 = (1/ki) * h;
    ad = kd / (kd + N * h);
    bd = kd * N / (kd + N * h); 
    //b = (b == 1) ? 1 : (1-k)/k; make sure there is always a feed forward from proportional control
    //condition ? value_if_true : value_if_false;
    // Anti-windup time constant
    //tt = h/(ki + 1);
}

sim::sim() 
 
:  h{0}, G{0}, gamma{0.8}, 
  log_R0{0}, Vi{0}, Vf{0}, k{0}, ref{0,0}, compute{false},
  tau_l{0}, tau_c{0}, lux_est{0}  { }

sim::~sim() {}

void sim::param_est(int _ref) {
    ref[1] = _ref;
    if (ref[0] != _ref) { 
        Vi = DAC_RANGE*R1/(R1 + R0*pow(ref[0],-gamma));
        //Serial.print("Vi "); Serial.println(Vi); 
        // Calculate every ref change 
        float r_ldr_final = R0*pow(ref[1],-gamma); //fnal est ldr
        float r_eq = R1*r_ldr_final/(R1 + r_ldr_final); // equivalent resistance 
        Vf = DAC_RANGE*R1/(r_ldr_final + R1); // estimate of final voltage read
        //Serial.print("Vf "); Serial.println(Vf); 
        tau_c = (r_eq * CAP1 * pow(10,-6));
        //Serial.print("tau_c "); Serial.println(tau_c); 
        if (Vf > Vi) {
            tau_l = (20/5/1000); // 20ms div 5, rise time
        }
        else {
            tau_l = (30/5/1000); // 30ms div 5, fall time/decay
        }
        //Serial.print("tau_l "); Serial.println(tau_l); 
        compute = true; 
        k = 0;
    }
    ref[0] = ref[1];
}   

float sim::sys_sim() { 
    if (compute && (k*h <= 8*(tau_l+tau_c))) {
        float vout;
        float r_ldr_est;
        // calculate till equilibrium, k*h >= 5*(tau_c + tau_l)
        //Serial.print("k*h "); Serial.println(k*h); 
        vout = Vf + (Vi - Vf)*exp((-k*h/(tau_c + tau_l)));
        r_ldr_est = R1*(DAC_RANGE - vout)/vout;
        //lux_est = pow(r_ldr_est/R0,-1/gamma);
        lux_est = pow(10,(log_R0 - log10(r_ldr_est))/gamma);
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
    ref[0] = _dist;
    log_R0 = log10(float(R0));
    lux_est = _dist;
}