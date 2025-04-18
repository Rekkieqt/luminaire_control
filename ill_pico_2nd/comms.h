#ifndef comms_H
#define comms_H
#include <Arduino.h>
#include "can.h"
#include "mcp2515.h"
#include "init.h"
#include "performance.h"
#include "ring_buffer.h"

uint16_t flt_to_16bit(float value);
float bits_to_flt(uint16_t encoded);

class canbus_comm
{
  private:
    String canintf_str = "| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | ";
    String eflg_str = "| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | ";
    id_data id;
    static can_data_decoder can_gut; // for msg send and recv modifications
    uint8_t myId{myIdentifier};
    uint16_t NUM_NODES{2};
    float * cxgains{nullptr};
    float * gmatrix{nullptr};
    //uint64_t can_data;
    //stream vars
    bool stream_check_u{false};
    bool stream_check_y{false};
    uint8_t stream_id;
  public:
    //constructor
    canbus_comm();
    //destructor
    ~canbus_comm();

    //functions
    void inner_frm_to_fifo(msg_to_can* inner_frame);    
    bool send_can(msg_to_can* inner_frame, MCP2515* can);
    bool recv_msg(msg_to_can* inner_frame);
    bool send_msg(msg_to_can* inner_frame, uint16_t canm_id, void* data = nullptr ,size_t size_data = 0); // (uint16_t id, uint64_t data, msg_to_can* inner_frame)
    void process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq);
    bool process_msg_core0(msg_to_can* inner_frame, data_reads* curr_data = nullptr, ser_data* info = nullptr, bool *stream_en = nullptr);  
    void error_process(msg_to_can* inner_frame, MCP2515* can);

    void ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd, void* val = nullptr, size_t bytes = 0);
    void ser_reply(msg_to_can* inner_frame, data_reads* curr_data, ser_data* info);
    void ser_receive(msg_to_can* inner_frame);

    void ntwrk_calibration(msg_to_can* inner_frame);
    void assign_cross_gain_vector();
    void set_ntwrk_params(MCP2515* can, uint8_t id = myIdentifier, uint8_t node_max = static_cast<uint8_t>(maxId + 1));
    void set_masks_n_filters(MCP2515* can);

    void opt_send(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_param, void* val, size_t bytes);
    void cross_gains_sync(msg_to_can* inner_frame);
};


#endif //comms_H
