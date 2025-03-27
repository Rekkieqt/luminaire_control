#include <Arduino.h>
#include "luxmeter.h"
#include "pid.h"
#include "ring_buffer.h"
#include "mcp2515.h"
#include "comms.h"
#include "can.h"
#include "init.h"

// Luminaire control init
volatile int x{0};
volatile int ref{0};
int usr_ref{0};
volatile int pwm{0};
int usr_pwm{0};
volatile bool ctrler_flag{false};

struct repeating_timer timer;
ring_buffer data_log; //last minute buffer of events
static_lux_data lux_data; // constant parameters of the box, dist, max lux and Gain
data_reads current_data, print_data; //data read inside the control sequence
data_reads sim_data , print_sim; //out simulated based on ref
luxmeter luxm(R0, Nfilter);
pid wallee(1.0f/Fs, 1 , 0.12 , 0.00, 100, 0.1, 0); //h kp ki kd tt b c (1.0f/Fs, 2 , 0.08 , 0.00, 500, 0.1, 0) good values but u flicker!!!!
sim observer;                                      //                   (1.0f/Fs, 1 , 0.12 , 0.00, 100, 0.1, 0) good values low flicker!!!!

//can bus init 
pico_unique_board_id_t pico_board_id; // Full ID
uint8_t node_id;                      // Short ID
MCP2515 canbuz {spi0, CSpin, TXpin, RXpin, SCKpin, SPIclock}; // canbus init 

uint64_t counter{0}; // incr on data send, has max size of can_bus frame
struct repeating_timer write_timer; // write repeating timer
volatile bool got_irq {false}; // receive msg flat
volatile bool time_to_write {false}; // receive msg flat
//inner frames , core to core msging
msg_to_can inner_frm_core0;
msg_to_can inner_frm_core1;
// message handler class
canbus_comm hermes;

// serial user input init
volatile bool usr_update{false};
performance measurer;
perf_meas curr_perf;
user_set_flags usr_control, curr_control;
msg_read usr_input;
serial_comm biden;
//volatile bool flag {false};
spin_lock_t *slk {0};

// can bus functions
bool timer_seq( struct repeating_timer *t ){ 
  time_to_write = true;
  return true;
}

//the interrupt service routine for core 1
void read_interrupt(uint gpio, uint32_t events) {
  got_irq = true;
}

// control sequence functions

bool control_seq( struct repeating_timer *t ){  
  //control sequence
  current_data.time = time_us_64()/1000; // uint 32 bit
  //current_data.ref = ref;
  current_data.out = luxm.get_lux();
  if (curr_control.pid){
    current_data.u = wallee.feed_backward(current_data.ref, current_data.out, lux_data.G, lux_data.min_lux, curr_control);
    wallee.housekeep(current_data.ref);  
  }
  else {
    current_data.u = pwm;
  }
  analogWrite(LED_PIN,(unsigned int)current_data.u);
  //data_log.push(&current_data);
  //observer 
  observer.param_est(current_data.ref);
  current_data.sim_out = observer.sys_sim();
  // square wave
  x = x + 1;
  if (current_data.ref == 20 && (x%800 == 0)) { //every 8 sec
    current_data.ref = 10; 
  }    
  else if (current_data.ref == 10 && (x%800 == 0) || current_data.ref == 0) { //every 8 sec
    current_data.ref = 20;
  }
  ctrler_flag = true;
  return true;
}

void setup() {
  // control and serial setup
  Serial.begin();
  analogReadResolution(DAC_RES); 
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  //luxm.tf_sweep();
  luxm.calibrate();
  luxm.get_lux_data(&lux_data);
  observer.init_sim(1.0f/Fs, lux_data.G, lux_data.min_lux);
  measurer.get_pmax();
  add_repeating_timer_ms( -10, control_seq, NULL, &timer); //100 Hz
  //can bus setup
  pico_get_unique_board_id(&pico_board_id); // might not be needed
  node_id = pico_board_id.id[7]; //check
  add_repeating_timer_ms( -1, timer_seq, NULL, &write_timer); //xx Hz
  //serial spinlock
  int slk_number = spin_lock_claim_unused( true );
  slk = spin_lock_init(slk_number);
}

void loop() {
  if(ctrler_flag) { //every control seq
    ctrler_flag = false;  
    data_log.push(&current_data);
    //data printing
    print_sim = sim_data;
    print_data = current_data;
    curr_perf = measurer.get_perf(print_data);
    //
    if (curr_control.rt_stream_out) {
      Serial.print(" Lux "); Serial.println(print_data.out);  
    }
    if (curr_control.rt_stream_u) {
      Serial.print(" Duty cycle on pwm "); Serial.println(print_data.u*100/DAC_RANGE);  
    }
    //TESTING LOGGING
    
    Serial.print(" "); Serial.print(print_data.time);
    Serial.print(" "); Serial.print(print_data.ref);
    Serial.print(" "); Serial.print(print_data.u*100/DAC_RANGE);
    Serial.print(" "); Serial.print(print_data.out);
    Serial.print(" "); Serial.println(print_data.sim_out); 
    /*
    */
  }
  if (usr_update) {
    uint32_t isr_state = spin_lock_blocking( slk );
    curr_control = usr_control;
    pwm = usr_pwm;
    ref = usr_ref;
    spin_unlock( slk, isr_state);
  }
  if(Serial.available()){
    size_t n_bytes = Serial.readBytes(usr_input.letter, MSG_SIZE - 1);
    usr_update = biden.decode(&usr_input, n_bytes, &luxm ,&current_data, &data_log, &lux_data, &usr_control, &curr_perf, usr_ref, usr_pwm);
  }
  if(time_to_write) {
    time_to_write = false;
    //comment for no sending 
    //hermes.send_msg(node_id,HEAD_FLAG,counter++,&inner_frm_core0);
  }
  hermes.recv_msg(&inner_frm_core0);
  hermes.process_msg_core0(&inner_frm_core0);
}

void setup1() {
  Serial.begin();
  canbuz.reset();
  canbuz.setBitrate(CAN_1000KBPS);
  canbuz.setNormalMode();
  //canbuz.setLoopbackMode();
  gpio_set_irq_enabled_with_callback( INTpin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
}

void loop1() {
  hermes.process_can_core1(&inner_frm_core1,&canbuz,got_irq);
  hermes.recv_msg(&inner_frm_core1);
  hermes.send_can(&inner_frm_core1,&canbuz); 
}