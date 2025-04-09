#ifndef comms_H
#define comms_H
#include <Arduino.h>
#include "can.h"
#include "comms.h"
#include "ring_buffer.h"
#include "mcp2515.h"
#include "init.h"
#include "performance.h"


class canbus_comm
{
  private:
    String canintf_str = "| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | ";
    String eflg_str = "| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | ";
    id_data id;
    static can_data_decoder can_gut; // for msg send and recv modifications
    uint8_t myId{0};
    uint8_t NUM_NODES{0};
    float * cxgains{nullptr};
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
    void assign_cross_gain_vector();
    bool send_can(msg_to_can* inner_frame, MCP2515* can);
    bool recv_msg(msg_to_can* inner_frame);
    bool send_msg(msg_to_can* inner_frame, uint16_t canm_id, void* data = nullptr ,size_t size_data = 0); // (uint16_t id, uint64_t data, msg_to_can* inner_frame)
    void process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq);
    bool process_msg_core0(msg_to_can* inner_frame, bool *stream_en = nullptr);  
    void error_process(msg_to_can* inner_frame, MCP2515* can);
    void ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd, void* val = nullptr, size_t bytes = 0);
    void ser_reply(msg_to_can* inner_frame);
    void ser_receive(msg_to_can* inner_frame);
    void ntwrk_calibration(msg_to_can* inner_frame);
    void set_ntwrk_params(uint8_t id = myIdentifier, uint8_t node_max = static_cast<uint8_t>(maxId + 1));
};


#endif //comms_H
