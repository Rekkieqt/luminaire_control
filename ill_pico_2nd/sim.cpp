#include <Arduino.h>
#include "init.h"
#include "ldr.h"
#include "sim.h"

sim::sim() 
 
:  h{0}, G{0}, Vi{0}, Vf{0}, k{0}, ref{0,0}, compute{false},
  tau_l{0}, tau_c{0}, lux_est{0} {
    get_ldr_params(params);
  }

sim::~sim() {}

void sim::param_est(float _ref) {
    ref[1] = _ref;
    if (ref[0] != _ref) { 
        Vi = DAC_RANGE*R1/(R1 + params[1]*pow(ref[0],-params[0]));
        // Calculate every ref change 
        float r_ldr_final = params[1]*pow(ref[1],-params[0]); //fnal est ldr
        float r_eq = R1*r_ldr_final/(R1 + r_ldr_final); // equivalent resistance 
        Vf = DAC_RANGE*R1/(r_ldr_final + R1); // estimate of final voltage read
        tau_c = (r_eq * CAP1 * pow(10,-6));
        if (Vf > Vi) {
            tau_l = (20/5/1000); // 20ms div 5, rise time
        }
        else {
            tau_l = (30/5/1000); // 30ms div 5, fall time/decay
        }
        compute = true; 
        k = 0;
    }
    ref[0] = ref[1];
}   

float sim::sys_sim() { 
    if (compute && (k*h <= 8*(tau_l+tau_c))) {
        // calculate till equilibrium, k*h >= 5*(tau_c + tau_l)
        float vout = Vf + (Vi - Vf)*exp((-k*h/(tau_c + tau_l)));
        float r_ldr_est = R1*(DAC_RANGE - vout)/vout;
        //lux_est = pow(r_ldr_est/params[1],-1/params[0]);
        lux_est = pow(10,(log_R0 - log10(r_ldr_est))/params[0]);
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
    log_R0 = log10(params[1]);
    lux_est = _dist;
    //Serial.print("gamma "); Serial.println(params[0]); 
    //Serial.print("log_R0 "); Serial.println(log_R0); 
    //Serial.print("R0 "); Serial.println(params[1]); 
}