#ifndef comms_H
#define comms_H
#include "can.h"
#include "luxmeter.h"
#include "comms.h"
#include "ring_buffer.h"
#include "mcp2515.h"
#include "init.h"

class serial_comm 
{
//inline defs
  private:
    msg_read decoder;
  public:
    //constructor
    serial_comm();
    //destructor
    ~serial_comm();

    //function
    void decode(void* user_input, int n_bytes, luxmeter &meas, data_reads &curr_data, ring_buffer &curr_buff);
    void func_test(int msg1 = 0, float msg2 = 0);   
};

class canbus_comm
{
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
