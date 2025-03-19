#include "luxmeter.h"
#include "pid.h"
#include "init.h"
#include "ring_buffer.h"

// Luminaire control init
volatile int x{0};
struct repeating_timer timer;
ring_buffer data_log;
static_lux_data lux_data;
data_reads current_data;
data_reads sim_data;
luxmeter luxm(R0, Nfilter);
pid wallee(1.0f/Fs, 40 , 5 , 0.01, 2); //kp 40 //ki 2
sim observer;

bool control_seq( struct repeating_timer *t ){  
  // control sequence
  current_data.time = millis();
  current_data.out = luxm.get_lux();
  current_data.u = wallee.feed_backward(current_data.ref, current_data.out, lux_data.G, lux_data.min_lux);
  analogWrite(LED_PIN,current_data.u);
  wallee.housekeep(current_data.ref);
  data_log.push(&current_data);
  Serial.print(" "); Serial.println(current_data.time);
  Serial.print(" "); Serial.println(current_data.ref);
  Serial.print(" "); Serial.println(current_data.out);
  observer.param_est(sim_data.ref);
  sim_data.out = observer.sys_sim();
  Serial.print(" "); Serial.println(sim_data.out);
  x = x + 1;
  if (current_data.ref == 25 && (x%5000 == 0)) {
    current_data.ref = 15; 
  }    
  else if (current_data.ref == 15 && (x%5000 == 0) || current_data.ref == 0) {
    current_data.ref = 25;
  }
  sim_data.ref = current_data.ref; 
  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin();
  analogReadResolution(DAC_RES);
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  //luxm.tf_sweep();
  luxm.calibrate();
  luxm.get_lux_data(&lux_data);
  observer.init_sim(1.0f/Fs, lux_data.G, lux_data.min_lux);
  add_repeating_timer_ms( -10, control_seq, NULL, &timer); //100 Hz
}

void loop() {
  if(Serial.available()){
    noInterrupts();
    data_log.print_buff();
    interrupts();
  }
}

void setup1() {

}

void loop1() {
  
}