#include "init.h"
#include "ldr.h"
#include <math.h>
#include <Arduino.h>

#define n_reads 30

float gamma_ = (CIRC_NUM == 1) ? 0.9545 :
               (CIRC_NUM == 2) ? 1.1441 : 
               0.0; 
float R0 = (CIRC_NUM == 1) ? static_cast<float>(300e3*pow(10,gamma_)) :
           (CIRC_NUM == 2) ? static_cast<float>(200e3*pow(10,gamma_)) : 
           0.0;

float G{0};
float d{0};

float get_ldr_voltage(int pinNumber){
  // returns the voltage read from the LDR ADC pin
  float V{0};
  float v{0};
  int cnt{0};
  for(int i=0; i<10; i++) if(v = analogRead(pinNumber)){V += v; cnt++;} 

  V = cnt>0 ? V/cnt : 0; // average voltage read

  return V*VCC/4095;
}

float get_ldr_resistance(float v){
  return _R*VCC/(v + 1e-5) - _R;
}

float luxmeter(float v){
  float Rl = get_ldr_resistance(v); // LDR Resistance (Ohm)
  return static_cast<float>(pow(R0/(Rl),static_cast<float>(1)/gamma_)); // Measured Iluminance (Lx)
}

void get_ldr_params(float cal_params[2]){
  cal_params[0] = gamma_;
  cal_params[1] = (CIRC_NUM == 1) ? (300000) :
                  (CIRC_NUM == 2) ? (200000) : 
                  0.0; 
}

void adjust_gain(void){
  // adjusts the background illuminance and gain of the environment

  float l1{0}, l2{0};
  float u1{0.2}, u2{0.8};

  analogWrite(LED_PIN, 0);
  delay(5000);

  while(l1 <= 0){l1 = luxmeter(get_ldr_voltage(LDR_PIN));} 

  d = l1;

  analogWrite(LED_PIN, static_cast<int>(u1*(DAC_RANGE-1)));
  delay(7000);
  while(l1 <= 0){l1 = luxmeter(get_ldr_voltage(LDR_PIN));}
  analogWrite(LED_PIN, static_cast<int>(u2*(DAC_RANGE-1)));
  delay(7000);
  while(l2 <= 0){l2 = luxmeter(get_ldr_voltage(LDR_PIN));}
  
  G = (l2-l1)/(u2-u1);

  // Serial.print("G: "); Serial.println(G);
  // Serial.print("d: "); Serial.println(d);
}