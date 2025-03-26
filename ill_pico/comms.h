#ifndef comms_H
#define comms_H
#include <Arduino.h>
#include "can.h"
#include "luxmeter.h"
#include "comms.h"
#include "ring_buffer.h"
#include "mcp2515.h"
#include "init.h"

class serial_comm 
{
//inline defs
//  private:
//    msg_read decoder;
  public:
    //constructor
    serial_comm();
    //destructor
    ~serial_comm();

    //function
    String cut_string(String ser_msg, int start_point, int n_chars);
    int get_int_from_str(String ser_msg);
    bool decode(msg_read* user_input, size_t n_bytes, luxmeter* lmeas, data_reads* curr_data, 
                ring_buffer* curr_buff, static_lux_data* lux_info, user_set_flags* usr_flags, perf_meas* perf_data,
                int& ref , int& pwm );
};

class canbus_comm
{
  private:
    String canintf_str = "| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | ";
    String eflg_str = "| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | ";
    uint64_t* can_data_ptr = nullptr;
  public:
    //constructor
    canbus_comm();
    //destructor
    ~canbus_comm();

    //functions
    void inner_frm_to_fifo(msg_to_can* inner_frame);
    void send_can(msg_to_can* inner_frame, MCP2515* can);
    void recv_msg(msg_to_can* inner_frame);
    void send_msg(uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame);
    void process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq);
    void process_msg_core0(msg_to_can* inner_frame);  
};


#endif //comms_H
