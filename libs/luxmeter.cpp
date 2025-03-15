#include "luxmeter.h"
#include "ring_buffer.h"
#include "init.h"
#include <Arduino.h>

luxmeter::luxmeter(
  int _R1, int N)
    : resistor{10000}, gamma{0.8}, vcc{DAC_RANGE}, ldr{0}, lux_offset{0.0},
      lux{0.0}, G{1.0}, N{5}, R1{_R1}, min_lux{0} {
    get_offset();  
}

float luxmeter::get_ldr() {
  int div_voltage {0};
  for (int i = 0; i < N ; i++) {
    div_voltage = analogRead(A0);
    Serial.print("voltage "); Serial.println(div_voltage); 
    ldr = ldr + (resistor*(vcc-div_voltage))/div_voltage;
  }
  ldr = ldr/N;
  Serial.print("ldr "); Serial.println(ldr); 
  Serial.print("G "); Serial.println(G); 
  Serial.print("dist "); Serial.println(min_lux); 
  return ldr;
}

void luxmeter::get_offset() {
  lux_offset = log10(R1) + gamma;
}

float luxmeter::get_lux() {
  ldr = get_ldr();
  lux = pow(10,( (lux_offset) - log10(ldr) )/gamma) ;
  return lux;
}

void luxmeter::tf_sweep() {

  for (int i = 0; i < DAC_RANGE; i = i+4) {
    //write to voltage pin
    analogWrite(LED_PIN,i);
    delay(50);
    ldr = get_ldr();
    lux = get_lux(); 
    
    //serial print pwm
    Serial.print(i); Serial.print (" ");
    //serial voltage read
    Serial.print(analogRead(A0)); Serial.print (" ");
    //serial print lux
    Serial.print(lux); Serial.print (" ");
    //serial print ldr
    Serial.print(ldr); Serial.println (" ");
    
  }
}

float luxmeter::calibrate() {
  float  max_lux{0};
  //write 0 get lux
  analogWrite(LED_PIN,0);
  delay(10000);
  min_lux = get_lux(); //div voltage is pin rading from arduino
  //write max value to get max lux
  Serial.print("min_lux "); Serial.print(min_lux); Serial.println (" ");
  analogWrite(LED_PIN,DAC_RANGE);
  delay(10000);
  max_lux = get_lux();
  Serial.print("max_lux "); Serial.print(max_lux); Serial.println (" ");
  G = (max_lux-min_lux)/DAC_RANGE; 
  Serial.print("Multiplier constant G "); Serial.print (" ");
  Serial.print(G); Serial.println (" ");
  return G;
  //serial print new G and calibration complete
}

float luxmeter::get_dist() {
  return min_lux;
}

performance::performance(int _Pmax)
: Pmax{_Pmax}, miu{0,0,0}, time{0,0}, ref{0,0}, beta{1.0f/DAC_RANGE},
      energy{0}, visibility{0}, flicker{0} {}

float performance::get_energy(data_reads dk) {
  time[1] = dk.time;
  energy = Pmax*miu[1]*beta*(time[1] - time[0]) + energy;
  time[0] = time[1];
  return energy;
}

float performance::get_visibility(data_reads dk) {
  //instead of waiting for delay() when ref k != ref k-1, initiate timer
  ref[1] = dk.ref;
  float vk {max(0,ref[1] - dk.out)}; 
  visibility = (vk + visibility)/2;
  ref[1] = ref[0];
  return visibility;
}

float performance::get_flicker(data_reads dk) {
  float fk{0};
  miu[2] = dk.u;
  float a {miu[2] - miu[1]};
  float b {miu[1] - miu[0]};
  if (a*b < 0){
    fk = abs(a) + abs(b);
  }
  else {
    fk = 0;
  }
  flicker = (flicker + fk)/2;
  miu[0] = miu[1];
  miu[1] = miu[2];
  return flicker;
}
