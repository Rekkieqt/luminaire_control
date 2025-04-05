#include <Arduino.h>
#include "comms.h"
#include "init.h"
#include "boot.h"

canbus_comm::canbus_comm() {
}

canbus_comm::~canbus_comm() {
    //delete[] cxgains;
}

void canbus_comm::inner_frm_to_fifo(msg_to_can* inner_frame) {
    for(int i = 0; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) { 
        // to securely read implement here the blocking version
        rp2040.fifo.push_nb(inner_frame->in_msg[i]);
    }
}

bool canbus_comm::recv_msg(msg_to_can* inner_frame) {
    if (rp2040.fifo.pop_nb(&inner_frame->in_msg[0])) {
        for (int i = 1; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) {
            // to securely read implement here the blocking version
            rp2040.fifo.pop_nb(&inner_frame->in_msg[i]);
        }   
        Serial.println("Caught inner frame!"); 
        return true;
    }
    return false;
}

bool canbus_comm::send_can(msg_to_can* inner_frame, MCP2515* can) {
    if (inner_frame->wrapped.internal_msg[0] == REQUEST) {
        can->sendMessage(&inner_frame->wrapped.can_msg);            
        //inner_frame->wrapped.internal_msg[0] = CAN_REG; //special flag for just error frm
        inner_frame->wrapped.internal_msg[1] = ACK; //read flag
        inner_frame->wrapped.internal_msg[2] = can->getInterrupts();
        inner_frame->wrapped.internal_msg[3] = can->getErrorFlags();
        //maybe do something with interrupts or errors
        error_process(inner_frame, can);
        return true;
    }
    return false;
}

bool canbus_comm::send_msg(msg_to_can* inner_frame, uint16_t canm_id, void* data, size_t data_size) { // (uint16_t id, uint64_t data, msg_to_can* inner_frame)
        inner_frame->wrapped.can_msg.can_id = canm_id;
        inner_frame->wrapped.can_msg.can_dlc = data_size; // versatile variant
        //memcpy(inner_frame->wrapped.can_msg.data, &data, sizeof(data));
        if (!sizeof(data)) {
            memcpy(inner_frame->wrapped.can_msg.data, data, data_size);
        }
        inner_frame->wrapped.internal_msg[0] = REQUEST; // unread flag
        //inner_frame->wrapped.internal_msg[1] = header; unused
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
        error_process(inner_frame, can);
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
    if (inner_frame->wrapped.internal_msg[0] == REQUEST) {
        //dest, source, size
        //memcpy(&can_data,inner_frame->wrapped.can_msg.data,sizeof(can_data));
        memcpy(&can_gut,inner_frame->wrapped.can_msg.data,inner_frame->wrapped.can_msg.can_dlc);
        // print
        Serial.println("Msg Broadcast!");
        Serial.print("Msg id ");Serial.println(inner_frame->wrapped.can_msg.can_id);
        Serial.print("Msg dlc ");Serial.println(inner_frame->wrapped.can_msg.can_dlc);
        Serial.print("Data ");Serial.println(can_gut.four_bytes);
        
        /*---------------------------------------- decode id ----------------------------------------*/
        decodeCanId(inner_frame->wrapped.can_msg.can_id, id.receiver, id.sender, id.header, id.header_flag);
        /*-------------------------------------------------------------------------------------------*/
        
        switch (id.header) {
            case BOOT:
                // do something
                break;
            case CALIBRATION:
                // do something
                break;
            case SER_COM:
                // ser_recv();
                // do something
                break;
            case REFERENCE:
                // do something
                break;
            case RESTART:
                // do something
                break;                    
            default:
                break;
                // do something
        }
        inner_frame->wrapped.internal_msg[0] = ACK; // read flag
    }
}


void canbus_comm::error_process(msg_to_can* inner_frame, MCP2515* can) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    // same for receiv or on send
    if (inner_frame->wrapped.internal_msg[3] & 0b11111000) {
        Serial.println("-----------------------------------------------------------------");
        Serial.println( canintf_str );
        Serial.print("| ");
        for (int bit = 7; bit >= 0; bit--) {
            Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[2], bit ) ? '1' : '0' ); Serial.print(" | ");
        }
        //if x inner_frame->internal_msg[2];
        Serial.println(" ");
        Serial.println("-----------------------------------------------------------------");
        Serial.println(eflg_str);
        Serial.print("| ");
        for (int bit = 7; bit >= 0; bit--) {
            Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[3], bit) ? '1' : '0'); Serial.print(" | ");
        }
        Serial.println(" ");
        Serial.println("-----------------------------------------------------------------");

        uint32_t err_time = time_us_64()/1000;

        can->clearRXnOVRFlags(); 
        can->clearInterrupts();

        while (time_us_64()/1000 - err_time < FIVE_SEC) { 
            //wait on everything except control seq...
        }
    }
    else {
        //inner_frame->wrapped.internal_msg[3] - > post process errors
        //inner_frame->wrapped.internal_msg[2] - > post process interrupts
        can->clearRXnOVRFlags(); 
        can->clearInterrupts();
    }    
}

// void assign_cross_gain_vector() {
//     float* cx_gains = new float[NUM_NODES];
//     memset(cx_gains,0 ,NUM_NODES*sizeof(cx_gains));
// }

// void canbus_comm::ntwrk_calibration(msg_to_can* inner_frame, void(*cal)() ) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
//     float u[2] = {0.8, 0.2};
//     float lux{0.0};
//     uint16_t can_id;
//     uint32_t curr_time = time_us_64()/1000;
//     for (int k : u) {
//         for (int n = 0 ; n < NUM_NODES ; n++) {
//             if (myNode == n) {
//                 //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
//                 can_id = encondeCanID(myNode,BROADCAST,CALIBRATION,REQ);
//                 //uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame
//                 send_msg(can_id,REQUEST,inner_frame); //find way to send null data 
// /*------------------------------- CALIBRATION SEQ ------------------------------------------------*/
//                 analogWrite(LED_PIN, static_cast<int>(u[k]*(DAC_RANGE-1)));

//                 while ((time_us_64/1000 - curr_time) < FIVE_SEC) {
//                     lux = luxmeter(get_ldr_voltage(LDR_PIN));                    
//                 } 
//                 can_id = encondeCanID(myNode,BROADCAST,CALIBRATION,ACK);
//                 send_msg(can_id,REQUEST,inner_frame);
//                 int k{0};
//                 while(k < NUM_NODES - 1 || (time_us_64()/1000 - last_restart) < TEN_SEC ) {
//                     if (recv_msg(inner_frame)) {
//                         decodeCanID(inner_frame->wrapped.can_msg.can_id,id.receiver.sender,id.receiver.receiver,id.header.sender,id.receiver.header_flag);
//                         k = (id.header_flag == ACK) ? ++k : k;
//                     }
//                 }
//                 analogWrite(LED_PIN, 0);
//                 cx_gains[n] -= lux/(u[0]-u[1]);
//             }
// /*-------------------------------------------------------------------------------------------------*/            
//             else {
//                 while((time_us_64/1000 - curr_time) < TEN_SEC) { 
//                     if(recv_msg(inner_frame)) {
//                         decodeCanID(inner_frame->msg_to_can.wrapped.can_msg.can_id, id.sender, id.receiver, id.header, id.header_flag);
//                         if (id.header_flag == REQ && id.sender == n ) break;
//                     }
//                 }
//                 //process_msg_core0(inner_frame);
//                 while (id.header_flag != ACK || (time_us_64()/1000 - last_restart) < TEN_SEC) {
//                     //cross calibration mynode -> current node
//                     lux = luxmeter(get_ldr_voltage(LDR_PIN));
//                     if(recv_msg(inner_frame)) {
//                         decodeCanID(inner_frame->msg_to_can.wrapped.can_msg.can_id, id.sender, id.receiver, id.header, id.header_flag);
//                     }
//                 }
//                 cx_gains[n] -= lux/(u[1]-u[2]);
//             }
//         }
//     }   
// }

// void canbus_comm::ser_req(uint8_t req_id, uint8_t req_cmd, msg_to_can* inner_frame) {
//     //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
//     uint16_t can_id = encondeCanID(myID,req_id,SER,REQ);    
//     //uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame
//     send_msg(can_id,req_cmd,inner_frame);
    
// }

// //decode outside, only go to ser recv 
// void canbus_comm::ser_recv(msg_to_can* inner_frame) {
//     uint16_t can_id = encondeCanID(id.receiver ,id.sender, SER, ACK);
//     send_msg(can_id,req_cmd,inner_frame);

//     switch (can_gut.bytes[0]) {
//         // Sets
//         case set_ref:
//             // handle set_ref
//             PID.set_reference(can_gut.bytes[1]);
//             break;
//         case set_u:
//             // handle set_u
//             set_dutycycle(can_gut.bytes[1]);
//             break;
//         case set_occ:
//             // handle set_occ
//             occ_st = occ_st ^ true;
//             ref = (occ_st == true) ? PID.r_h : PID.r_l;
//             PID.set_reference(ref);
//             break;
//         case set_aa:
//             // handle set_occ
//             PID.set_anti_wu_status(can_gut.bytes[1]);
//             break;            
//         case set_fb:
//             // handle set_occ
//             PID.set_fb_status(can_gut.bytes[1]);
//             break;            

//         // Gets
//         case get_ref:
//             // handle get_ref
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, PID.get_reference() ,inner_frame);
//             break;
//         case get_u:
//             // handle get_u
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, u ,inner_frame);
//             break;
//         case get_y:
//             // handle get_y
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, L ,inner_frame);
//             break;
//         case get_volt:
//             // handle get_volt
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, v ,inner_frame);
//             break;
//         case get_occ:
//             // handle get_occ
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, occ_st ,inner_frame);
//             break;
//         case get_aa:
//             // handle get_aa
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, PID.get_anti_wu_status() ,inner_frame);
//             break;
//         case get_fb:
//             // handle get_fb
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, PID.get_anti_fb_status() ,inner_frame);
//             break;
//         case get_dist:
//             // handle get_dist
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, PID.get_disturbance() ,inner_frame);
//             break;
//         case get_pwr:
//             // handle get_pwr
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, inst_power_consumption(uk_1) ,inner_frame);
//             break;
//         case get_Rtime:
//             // handle get_Rtime
//             {
//                 uint32_t elapsed_time = time_us_64()/1000 - last_restart;
//                 can_id = encondeCanID(myID ,id.sender, SER, ACK);
//                 send_msg(can_id, elapsed_time ,inner_frame);
//             }
//             break;
//         case get_buff_u:
//             // handle get_buff_u
//             //to do
//             break;
//         case get_buff_y:
//             // handle get_buff_y
//             //to do
//             break;

//         // Performance metrics
//         case get_flicker:
//             // handle get_flicker
//             {
//                 float flicker_avg = flicker/N;
//                 can_id = encondeCanID(myID ,id.sender, SER, ACK);
//                 send_msg(can_id, flicker_avg ,inner_frame);
//             }
//             break;
//         case get_vis:
//             // handle get_vis
//             {
//                 float vis_avg = vis/N;
//                 can_id = encondeCanID(myID ,id.sender, SER, ACK);
//                 send_msg(can_id, vis_avg ,inner_frame);
//             }
//             break;
//         case get_energy:
//             // handle get_energy
//             send ener
//             can_id = encondeCanID(myID ,id.sender, SER, ACK);
//             send_msg(can_id, ener ,inner_frame);
//             break;

//         // Streams
//         case start_stream_u:
//             // handle start_stream_u
//             //to do
//             break;
//         case start_stream_y:
//             // handle start_stream_y
//             //to do
//             break;
//         case stop_stream_u:
//             // handle stop_stream_u
//             //to do
//             break;
//         case stop_stream_y:
//             // handle stop_stream_y
//             //to do
//             break;

//         // Distribution
//         case get_lower_bound_occ:
//             // handle get_lower_bound_occ
//             break;
//         case set_lower_bound_occ:
//             // handle set_lower_bound_occ
//             break;
//         case get_lower_bound_unocc:
//             // handle get_lower_bound_unocc
//             break;
//         case set_lower_bound_unocc:
//             // handle set_lower_bound_unocc
//             break;
//         case get_curr_cost:
//             // handle get_curr_cost
//             break;
//         case get_curr_lum:
//             // handle get_curr_lum
//             break;
//         case set_curr_cost:
//             // handle set_curr_cost
//             break;

//         // Reset
//         case reset:
//             // handle reset
//             occ_st = UNOCC;
//             N = 0;
//             ener = 0;
//             vis_err = 0;
//             flicker = 0;
//             _data_log->clear();
//             last_restart = time_us_64()/1000;
//             adjust_gain();
//             PID.set_reference(occ_st ? PID.r_h : PID.r_l);
//             PID.set_system_gain(G);
//             break;
//         default:
//             // handle unknown request
//             Serial.println("err");
//             break;
//     }
// }

