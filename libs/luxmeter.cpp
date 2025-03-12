#include "luxmeter.h"
#include <Arduino.h>
#define DAC_RANGE 4095
#define LED_PIN 15

luxmeter::luxmeter(
  int _R1)
    : resistor{10000}, gamma{0.8}, vcc{DAC_RANGE}, ldr{0}, lux_offset{0.0},
      lux{0.0}, G{0.0}, N{5}, R1{_R1} {
    get_offset();  
}

double luxmeter::get_ldr() {
  int div_voltage {0};
  for (int i; i < N ; i++) {
    div_voltage = analogRead(A0);
    ldr = ldr + (resistor*(vcc-div_voltage))/div_voltage;
  }
  ldr = ldr/N;
  return ldr;
}

void luxmeter::get_offset() {
  lux_offset = log10(R1) + gamma;
}

double luxmeter::get_lux() {
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

double luxmeter::calibrate() {

  double input_range {DAC_RANGE}, min_lux{0}, max_lux{0};
  //write 0 get lux
  analogWrite(LED_PIN,0);
  delay(1000);
  min_lux = get_lux(); //div voltage is pin rading from arduino
  //write max value to get max lux
  Serial.print("min_lux "); Serial.print(min_lux); Serial.println (" ");
  analogWrite(LED_PIN,DAC_RANGE);
  delay(1000);
  max_lux = get_lux();
  Serial.print("max_lux "); Serial.print(max_lux); Serial.println (" ");
  G = (max_lux-min_lux)/input_range; 
  Serial.print("Multiplier constant G "); Serial.print (" ");
  Serial.print(G); Serial.println (" ");
  return G;
  //serial print new G and calibration complete
}

performance::performance(int _Pmax)
: Pmax{_Pmax}  

double performance::get_energy() {
  
}

double performance::get_visibility() {
  lux_offset = log10(R1) + gamma;
}

double performance::get_flicker() {
  lux_offset = log10(R1) + gamma;
}
