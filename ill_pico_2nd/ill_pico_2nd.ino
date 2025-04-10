#include <Arduino.h>
#include <cstdio>
#include "init.h"
#include "ldr.h"
#include "pid.h"
#include "sim.h"
#include "performance.h"
#include "ring_buffer.h"
#include "can.h"
#include "mcp2515.h"
#include "comms.h"
#include "boot.h"
#include "command.h"
#include "optm.h"

uint8_t myIdentifier{0};
uint8_t maxId{0};

/*---------- GLOBAL VARIABLES ----------*/
// pid PID {static_cast<float>(SAMPLE_TIME)*1e-3, 1.5, 6, 0, 0.07, 0.05, 0.025, 0.003}; // PID controller initialization
pid PID {static_cast<float>(SAMPLE_TIME)*1e-3, 15e8, 6e8, 0,8e-10, 7e-10, 4e-10, 0.003}; // PID controller initialization
ring_buffer data_log; //last minute buffer of events
data_reads current_data, print_data; //data read inside the control sequence
ser_data serial_info; // for parsing data into hub network
sim observer;
optimizer nice;

bool occ_st{UNOCC}; // occupancy state

float ener{0}; // performance metrics
float flicker{0};
float vis_err{0};
unsigned long N{0};
unsigned long last_restart{0};

bool print_y{false};
bool print_u{false};
bool print_buff{false};
bool SERIAL_PRINTS_0{false};
bool SERIAL_PRINTS_1{false};
bool dn_special_print{false};
bool stream_delay{false};

float v{0};
float uk_1{0};
float uk_2{0};

char x{'\0'};
bool printing{false};
/*-----------------------------*/

/*---------- PID INTERRUPT FLAG N TIMER ----------*/
volatile bool ctrler_flag{false};
struct repeating_timer pid_timer;
/*-----------------------------*/

/*---------- CAN BUS AND FIFO FRAMES ----------*/
MCP2515 canbuz {spi0, CSpin, TXpin, RXpin, SCKpin, SPIclock}; // canbus init 
volatile bool got_irq {false}; // receive msg flat
canbus_comm hermes;
msg_to_can inner_frm_core0;
msg_to_can inner_frm_core1;

/*---------- CAN BUS READ ON MSG RECEPTION ----------*/
void read_interrupt(uint gpio, uint32_t events) {
  got_irq = true;
}
// temp 
volatile bool time_to_write {false}; // receive msg flat
uint64_t counter{0}; // incr on data send, has max size of can_bus frame
struct repeating_timer write_timer; // write repeating timer
// can bus functions
bool timer_seq( struct repeating_timer *t ){ 
  time_to_write = true;
  return true;
}
/*-----------------------------*/
/*---------- SPINLOCK / MUTEX ----------*/
//empty for now

/*---------- CONTROL INTERRUPT CALLBACK ----------*/

bool control_seq( struct repeating_timer *t ){  
  current_data.time = time_us_64()/1000; // uint 32 bit
  current_data.ref = PID.get_reference();
  observer.param_est(PID.get_reference());
  current_data.sim_out = observer.sys_sim();
  v = get_ldr_voltage(LDR_PIN);
  current_data.out = filter(luxmeter(v)); // READ AND FILTER
  current_data.u = PID.computePWM(current_data.out);
  PID.housekeep(current_data.out);
  analogWrite(LED_PIN,current_data.u*(DAC_RANGE-1));
  
  ctrler_flag = true;
  return true;
}
/*-----------BOOT------------*/
boot booty;


/*-----------------------------*/

/*---------- LOW-PASS FILTER ----------*/

float filter(float u){
  static float u_prev{0};
  static float y_prev{0};
  const float b0{0.2391}, b1{0.2391}, a1{-0.5219}; // low-pass at 10Hz
  float y = b0*u + b1*u_prev - a1*y_prev;
  u_prev = u;
  y_prev = y;
  return y;
}
/*-----------------------------*/

void setup() {
  Serial.begin();
  last_restart = time_us_64()/1000; // uint 32 bit
  analogReadResolution(DAC_RES); 
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  delay(3000);
  booty.NODE_BOOT(&hermes, &inner_frm_core0);
  myIdentifier = booty.nodeId;
  maxId = booty.num_nodes - 1;
  hermes.set_ntwrk_params(&canbuz);
  hermes.ntwrk_calibration(&inner_frm_core0);
  hermes.cross_gains_sync(&inner_frm_core0);
  /*---------- GAIN AND PID PARAMETERS ----------*/
  //adjust_gain();
  PID.set_system_gain_n_dist(G, d);
  PID.set_reference(occ_st ? PID.r_h : PID.r_l);
  
  /*---------- SYS SIM ----------*/
  observer.init_sim(1.0f/Fs, G, d);
  
  /*---------- CONTROL INT SETUP ----------*/
  add_repeating_timer_ms( -SAMPLE_TIME, control_seq, NULL, &pid_timer); //100 Hz

  nice.set_cnstr_fn(maxId + 1);
  float A[4] = {27.2, 9.6, 16.6, 3.6};
  nice.set_constraints(A, PID.get_reference()-d);
  float u{0};
  nice.iterate_primal(u);
  hermes.send_msg(&inner_frm_core0, encodeCanId(myIdentifier,BROADCAST,OPTIMIZATION,INPUT_U), &u, sizeof(u));
}

void loop() {
  if(ctrler_flag) { //every control seq
    ctrler_flag = false;  
    stream_delay = true;
    /*---------- DATA SYNC ----------*/
    //data printing
    uint32_t irq_state = save_and_disable_interrupts();
    print_data = current_data;
    restore_interrupts(irq_state);
    data_log.push(&print_data);
    
    /*---------- SERIAL PRINTING ----------*/
    if(print_buff) {
      data_log.print_buff(x,CIRC_NUM); // 1 MIN BUFFER PRINTING
      print_buff = false;  // only once per every serial call
    } 
    if(dn_special_print) {
      Serial.print(" "); Serial.print(print_data.time);
      Serial.print(" "); Serial.print(print_data.ref);
      Serial.print(" "); Serial.print(print_data.u);
      Serial.print(" "); Serial.print(print_data.out);
      Serial.print(" "); Serial.println(print_data.sim_out); 
      }
    if(SERIAL_PRINTS_0){
      Serial.printf("t:%lu ",print_data.time);
      Serial.printf("r:%.2f ",print_data.ref);
      Serial.printf("y:%.4f ",print_data.out);
      Serial.printf("u:%.4f ",print_data.u);
      Serial.printf("0:%d ", 0);
      Serial.printf("Lt_s:%.4f\n",print_data.sim_out);
    }
    if(print_u){Serial.printf("s u %d %f %lu\n", CIRC_NUM, print_data.u, print_data.time);}
    if(print_y){Serial.printf("s y %d %f %lu\n", CIRC_NUM, print_data.out, print_data.time);}

    if(N > 0) {
      update_power_consumption(inst_power_consumption(uk_1)*(SAMPLE_TIME)*1e-3); // posso calcular o tempo exato de sample time, mas em ms deve ser sempre 10
    }
    if(N > 1) {
      update_flicker(print_data.u, uk_1, uk_2);
    }
    update_vis_err(print_data.out, PID.get_reference());

    uk_2 = uk_1;
    uk_1 = print_data.u;
    N++;

  /*------------------------------ SERIAL DATA SYNC ------------------------------*/
    serial_info.N = N;
    serial_info.voltage = v;
    serial_info.energy = ener;
    serial_info.visibility = vis_err;
    serial_info.flicker = N;
    /*----------------------------------------------------------------------------*/
  }

  /*------------------------------ SERIAL RECEIVER ------------------------------*/

  if(Serial.available()) get_command(v, print_data.out, print_data.u, ener, flicker, vis_err, N, &data_log, &hermes, &inner_frm_core0);
/*--------------------------------------------------------------------*/
  hermes.recv_msg(&inner_frm_core0);
  hermes.process_msg_core0(&inner_frm_core0, &print_data, &serial_info , &stream_delay);
}

void setup1() {
  Serial.begin();
  canbuz.reset();
  canbuz.setBitrate(CAN_1000KBPS);
  // /*-------------------- MASK AND FILTER SET--------------------*/
  // canbuz.setFilterMask(MCP2515::MASK0, 0, ID_MASK);
  // //uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag
  // canbuz.setFilter(MCP2515::RXF0, 0, (uint32_t)encodeCanId(0,BROADCAST,0,0));
  // canbuz.setFilter(MCP2515::RXF1, 0, (uint32_t)encodeCanId(0,myIdentifier,0,0)); 
  // canbuz.setFilterMask(MCP2515::MASK1, 0, ID_MASK);
  // canbuz.setFilter(MCP2515::RXF3, 0, (uint32_t)encodeCanId(0,BROADCAST,0,0)); 
  // canbuz.setFilter(MCP2515::RXF2, 0, (uint32_t)encodeCanId(0,myIdentifier,0,0));
  // /*---------------------------------------------------------*/
  canbuz.setNormalMode();
  //canbuz.setLoopbackMode();

  /*---------- CONTROL INT SETUP ----------*/
  gpio_set_irq_enabled_with_callback( INTpin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
}

void loop1() {

  /*---------- CAN BUS RECEPTION AND SENDING ----------*/
  hermes.process_can_core1(&inner_frm_core1, &canbuz, got_irq);
  hermes.recv_msg(&inner_frm_core1);
  hermes.send_can(&inner_frm_core1, &canbuz); 
}