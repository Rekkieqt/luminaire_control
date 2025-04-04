#include <Arduino.h>
#include "comms.h"
#include "init.h"

canbus_comm::canbus_comm() {
}

canbus_comm::~canbus_comm() {
    //delete[] cxgains;
}

void canbus_comm::inner_frm_to_fifo(msg_to_can* inner_frame) {
    for(int i = 0; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) { 
        rp2040.fifo.push_nb(inner_frame->in_msg[i]);
    }
}

bool canbus_comm::recv_msg(msg_to_can* inner_frame) {
    if (rp2040.fifo.pop_nb(&inner_frame->in_msg[0])) {
        for (int i = 1; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) {
        rp2040.fifo.pop_nb(&inner_frame->in_msg[i]);
        }   
        Serial.println("Caught inner frame!"); 
        return true;
    }
    return false;
}

bool canbus_comm::send_can(msg_to_can* inner_frame, MCP2515* can) {
    if (inner_frame->wrapped.internal_msg[0] == REQUEST){
        can->sendMessage(&inner_frame->wrapped.can_msg);            
        inner_frame->wrapped.internal_msg[0] = ERR_REQ; //special flag for just error frm
        inner_frame->wrapped.internal_msg[2] = can->getInterrupts();
        inner_frame->wrapped.internal_msg[3] = can->getErrorFlags();
        if (inner_frame->wrapped.internal_msg[3] & 0b11111000) {
            //inner_frame->wrapped.internal_msg[1] = CRITICAL_ERRORS;
            can->clearRXnOVRFlags(); 
            can->clearInterrupts();
        }
        inner_frm_to_fifo(inner_frame);//não devia ser esta a madar o true? porque a fifo pode estar cheia 
        inner_frame->wrapped.internal_msg[0] = ACK;
        return true;
    }
    return false;
}

bool canbus_comm::send_msg(uint16_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame) {
    inner_frame->wrapped.can_msg.can_id = id;
    inner_frame->wrapped.can_msg.can_dlc = sizeof(data);
    memcpy(inner_frame->wrapped.can_msg.data, &data, sizeof(data));
    inner_frame->wrapped.internal_msg[0] = REQUEST; // unread flag
    inner_frame->wrapped.internal_msg[1] = header;
    inner_frame->wrapped.internal_msg[2] = sizeof(can_frame);
    //inner_frame.internal_msg[2] = 0; extra 
    inner_frm_to_fifo(inner_frame);
    //Serial.println("Sent inner frame!");
    inner_frame->wrapped.internal_msg[0] = ACK; // 
    return true;//aqui a mesma coisa
}

void canbus_comm::process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    if (_got_irq) {
        _got_irq = false;
        inner_frame->wrapped.internal_msg[0] = REQUEST; // unread flag
        //inner_frame->wrapped.internal_msg[1] = READ; // inner frame header 
        inner_frame->wrapped.internal_msg[2] = can->getInterrupts();  //irq
        inner_frame->wrapped.internal_msg[3] = can->getErrorFlags(); //errors

        if(inner_frame->wrapped.internal_msg[3] & 0b11111000) {
            inner_frame->wrapped.internal_msg[1] = CRITICAL_ERRORS;
            //maybe something extra
            can->clearRXnOVRFlags(); 
            can->clearInterrupts();
        }
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX0IF) {
            can->readMessage( MCP2515::RXB0, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            Serial.println("Caught can bus message RXB0!");
            }
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX1IF) {
            can->readMessage( MCP2515::RXB1, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            Serial.println("Caught can bus message RXB1!");
            }
        inner_frame->wrapped.internal_msg[0] = ACK; // read flag
    } 
}

void canbus_comm::process_msg_core0(msg_to_can* inner_frame) { //receive can thru fifo, process data, do something on necessity
        switch (inner_frame->wrapped.internal_msg[0]) {
          case REQUEST: 
            //dest, source, size
            memcpy(&can_data,inner_frame->wrapped.can_msg.data,sizeof(can_data));
            // print
            Serial.println("Msg Broadcast!");
            Serial.print("msg id ");Serial.println(inner_frame->wrapped.can_msg.can_id);
            Serial.print("msg dlc ");Serial.println(inner_frame->wrapped.can_msg.can_dlc);
            Serial.print("data ");Serial.println(can_data);
            inner_frame->wrapped.internal_msg[0] = ACK; // read flag
            break;
          case ACK:
            //nop nop nop
            break;
          default: // space for other messages
            // do something 
            break;
        }
        switch (inner_frame->wrapped.internal_msg[1]) {
            case CRITICAL_ERRORS:
            //if x inner_frame->internal_msg[1];
            Serial.println("-----------------------------------------------------------------");
            Serial.println( canintf_str );
            Serial.print("| ");
            for (int bit = 7; bit >= 0; bit--) {
                Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[2], bit ) ? '1' : '0' ); Serial.print(" | ");
            }
            //if x inner_frame->internal_msg[2];
            Serial.println("");
            Serial.println("-----------------------------------------------------------------");
            Serial.println(eflg_str);
            Serial.print("| ");
            for (int bit = 7; bit >= 0; bit--) {
                Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[3], bit) ? '1' : '0'); Serial.print(" | ");
            }
            Serial.println("");
            Serial.println("-----------------------------------------------------------------");
            inner_frame->wrapped.internal_msg[1] = ACK; //read flag
            break;
            case ERR_REQ:
                inner_frame->wrapped.internal_msg[1] = ACK;
                // do something
                break;
            case ACK:
                // do something
                break;
            default:
                break;
              // do something
        }
}

/*
void assign_cross_gain_vector() {
    float* cx_gains = new float[NUM_NODES-1];
}

void canbus_comm::ntwrk_calibration(msg_to_can* inner_frame, void(*func)() ) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    for (int n = 0 ; n < NUM_NODES ; n++) {
        if (myNode == n) {
            //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
            uint16_t can_id = encondeCanID(myNode,BROADCAST,CALIBRATION,REQ);
            //uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame
            send_msg(can_id,REQUEST,0,inner_frame);

            //calibration routine routine x
            func();
            //end of calibration routine  x
            uint16_t can_id = encondeCanID(myNode,BROADCAST,CALIBRATION,ACK);
        }
        else {
            uint16_t can_id;
            while(!recv_msg(inner_frame) || (micro_us_64/1000 - last_restart)< TEN_SEC) { 
                //active wait...
            }
            process_msg_core0(inner_frame);
            while (can_id.header_flag != ACK || (micro_us_64/1000 - last_restart)< TEN_SEC) {

            //cross calibration mynode -> current node
            cx_gains[n-1] = luxmeter(get_ldr_voltage(LDR_PIN));

            if(recv_msg(inner_frame)) {
               decodeCanID(inner_frame->msg_to_can.wrapped.can_msg.can_id,can_id->sender,can_id->sender,can_id->header,can_id->header_flag);        

                }
            }
        }
    }
}

void canbus_comm::ser_req(uint8_t req_id, uint8_t req_cmd, msg_to_can* inner_frame) {
    //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
    uint16_t can_id = encondeCanID(myID,req_id,SER,REQ);
    //uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame
    send_msg(can_id,req_cmd,inner_frame);
}
//decode outside, only go to ser recv 
void canbus_comm::ser_recv(msg_to_can* inner_frame) {
    decodeCanID(inner_frame->msg_to_can.wrapped.can_msg.can_id,can_id->sender,can_id->sender,can_id->header,can_id->header_flag);        
    //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
    uint16_t can_id = encondeCanID(myID,req_id,SER,REQ);
    //uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame
    send_msg(can_id,req_cmd,inner_frame);
}
*/