#include <Arduino.h>
#include "init.h"
#include "ldr.h"
#include "sim.h"

sim::sim() 
 
:  h{0}, G{0}, gamma{0.8}, 
  log_R0{0}, Vi{0}, Vf{0}, k{0}, ref{0,0}, compute{false},
  tau_l{0}, tau_c{0}, lux_est{0}, R0_ldr {275000} {

    float* params = get_ldr_params();
    gamma = params[0];
    R0_ldr = params[1];

    delete[] params;
  }

sim::~sim() {}

void sim::param_est(float _ref) {
    ref[1] = _ref;
    if (ref[0] != _ref) { 
        Vi = DAC_RANGE*R1/(R1 + R0_ldr*pow(ref[0],-gamma));
        //Serial.print("Vi "); Serial.println(Vi); 
        // Calculate every ref change 
        float r_ldr_final = R0_ldr*pow(ref[1],-gamma); //fnal est ldr
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
        //lux_est = pow(r_ldr_est/R0_ldr,-1/gamma);
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
    log_R0 = log10(float(R0_ldr));
    lux_est = _dist;
}