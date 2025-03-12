#include "luxmeter.h"
#include <Arduino.h>
#define DAC_RANGE 4095
#define LED_PIN 15

luxmeter::luxmeter() {
  //int resistor, float gamma, float vcc, double ldr, double lux_offset, double lux, double G
  resistor = 10000;      
  gamma = 0.8;         
  vcc = DAC_RANGE; //corresponds to max voltage 3.3 V           
  ldr = 0;             
  lux_offset = 0.0;    
  lux = 0.0;           
  G = 0.0;            
}

luxmeter::~luxmeter() {
  
}

double luxmeter::get_ldr() {
  int div_voltage{analogRead(A0)};
  ldr = (resistor*(vcc-div_voltage))/div_voltage;
  return ldr;
}

void luxmeter::get_offset(int R1) {
  //dumb???
  int counter{0};
  while (( get_ldr() < R1 ) && (counter <= DAC_RANGE)) {
    //write to voltage pin
    analogWrite(LED_PIN,counter);
    delay(25);
    counter = counter + 2; //till 4095
    //serial print pwm
    Serial.print(counter); Serial.print (" ");
    //serial voltage read
    Serial.print(analogRead(A0)); Serial.print (" ");
    //serial print lux
    Serial.print(get_ldr()); Serial.print (" ");
    //serial print ldr
  }
  //lux_offset = log10(get_ldr(analogRead(A0))) + gamma;
  //or
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

void luxmeter::calibrate() {

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
  //serial print new G and calibration complete
}
