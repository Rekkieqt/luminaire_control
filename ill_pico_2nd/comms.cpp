#include <Arduino.h>
#include "performance.h"
#include "comms.h"
#include "init.h"
#include "boot.h"
#include "ldr.h"
#include "pid.h"
#include "command.h"

uint16_t flt_to_bits(float value) {
    static const float SCALE_FACTOR = 10000.0f;
    return static_cast<uint16_t>(value * SCALE_FACTOR);
}

float bits_to_flt(uint16_t encoded) {
  static const float SCALE_FACTOR = 10000.0f;
    return static_cast<float>(encoded) / SCALE_FACTOR;
}

canbus_comm::canbus_comm() {
    //
}

canbus_comm::~canbus_comm() {
    delete[] cxgains;
}

can_data_decoder canbus_comm::can_gut;

void canbus_comm::set_ntwrk_params(MCP2515* can, uint8_t id, uint8_t node_max){
    myId = id;
    NUM_NODES = node_max;
    set_masks_n_filters(can);
    assign_cross_gain_vector();
}

void canbus_comm::set_masks_n_filters(MCP2515* can) {
  /*-------------------- MASK AND FILTER SET--------------------*/
  can->reset();
  can->setBitrate(CAN_1000KBPS);
  can->setFilterMask(MCP2515::MASK0, 0, ID_MASK);
  //uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag
  can->setFilter(MCP2515::RXF0, 0, (uint32_t)encodeCanId(0,BROADCAST,0,0));
  can->setFilter(MCP2515::RXF1, 0, (uint32_t)encodeCanId(0,myId,0,0)); 
  can->setFilterMask(MCP2515::MASK1, 0, ID_MASK);
  can->setFilter(MCP2515::RXF3, 0, (uint32_t)encodeCanId(0,BROADCAST,0,0)); 
  can->setFilter(MCP2515::RXF2, 0, (uint32_t)encodeCanId(0,myId,0,0));
  can->setNormalMode();
  /*---------------------------------------------------------*/
}


void canbus_comm::opt_send(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_param, void* val, size_t bytes) {
    memcpy(&can_gut.floats[0], val, bytes);
    uint16_t can_id = encodeCanId(myId, req_id, OPTIMIZATION, req_param);    
    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut)); // - sizeof(uint8_t) - bytes);
}

void canbus_comm::assign_cross_gain_vector() {
    cxgains = new float[NUM_NODES];
    memset(cxgains, 0 , NUM_NODES*sizeof(float));
}

void canbus_comm::inner_frm_to_fifo(msg_to_can* inner_frame) {
    for (int i = 0; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) { 
        // to securely read implement here the blocking version
        //rp2040.fifo.push_nb(inner_frame->in_msg[i]);
        rp2040.fifo.push(inner_frame->in_msg[i]);
    }
}

bool canbus_comm::recv_msg(msg_to_can* inner_frame) {
    if (rp2040.fifo.pop_nb(&inner_frame->in_msg[0])) {
        for (int i = 1; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) {
            // to securely read implement here the blocking version
            //rp2040.fifo.pop_nb(&inner_frame->in_msg[i]);
            inner_frame->in_msg[i] = rp2040.fifo.pop();
        }   
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
        inner_frame->wrapped.internal_msg[0] = ACK;       
        can_data_decoder can_gut2;
        memcpy(&can_gut2, inner_frame->wrapped.can_msg.data, sizeof(can_gut));
        return true;
    }
    return false;
}

bool canbus_comm::send_msg(msg_to_can* inner_frame, uint16_t canm_id, void* data, size_t data_size) { // (uint16_t id, uint64_t data, msg_to_can* inner_frame)
        inner_frame->wrapped.can_msg.can_id = canm_id;
        inner_frame->wrapped.can_msg.can_dlc = data_size; // versatile variant
        //memcpy(inner_frame->wrapped.can_msg.data, &data, sizeof(data));
        if (data_size) {
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
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX0IF) {
            can->readMessage( MCP2515::RXB0, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            //Serial.println("Caught can bus message RXB0!");
            }
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX1IF) {
            can->readMessage( MCP2515::RXB1, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            //Serial.println("Caught can bus message RXB1!");
            }
        error_process(inner_frame, can);
        inner_frame->wrapped.internal_msg[0] = ACK; // read flag
    } 
}

bool canbus_comm::process_msg_core0(msg_to_can* inner_frame, data_reads* curr_data, ser_data* info, bool *stream_en) { //receive can thru fifo, process data, do something on necessity
    bool recv{false};
    if (inner_frame->wrapped.internal_msg[0] == REQUEST) {
        recv = true;
        //dest, source, size
        //memcpy(&can_data,inner_frame->wrapped.can_msg.data,sizeof(can_data));
        memcpy(&can_gut,inner_frame->wrapped.can_msg.data,inner_frame->wrapped.can_msg.can_dlc);
        // print
        /*---------------------------------------- decode id ----------------------------------------*/

        decodeCanId(inner_frame->wrapped.can_msg.can_id, id.sender, id.receiver, id.header, id.header_flag);

        /*-------------------------------------------------------------------------------------------*/
        // Serial.print("Sender Id ");Serial.println(id.sender,HEX);
        // Serial.print("Receiver Id ");Serial.println(id.receiver,HEX);
        // Serial.print("Header ");Serial.println(id.header,HEX);
        // Serial.print("Header flag ");Serial.println(id.header_flag,HEX);
        // Serial.print("Msg dlc ");Serial.println(inner_frame->wrapped.can_msg.can_dlc);
        switch (id.header) {
            case BOOT:
                // do something
                break;
            case CALIBRATION:
                // do something
                break;
            case SER_COM:
                // ser_recv();
                if (id.header_flag == GETS || id.header_flag == SETS) {
                    ser_reply(inner_frame, curr_data, info);
                }
                else if (id.header_flag == ACK) {
                    ser_receive(inner_frame);
                }
                memset(&can_gut, 0 , sizeof(can_gut));
                // do something
                break;
            case OPTIMIZATION:
                // required float at can_gut.floats[0];
                // do something
                // set read flag and memcopy the values...
                // call a function and pass it a value...
                // return uint8_t for specified reads...
                break; 
            case STREAM:
                //memcpy(&can_gut.floats[0],inner_frame->wrapped.can_msg.data,inner_frame->wrapped.can_msg.can_dlc);
                if(id.header_flag == LUX){
                    
                    Serial.print("Lux stream : "); Serial.println(can_gut.floats[0]);
                    Serial.print("From id : "); Serial.println(id.sender);
                }
                else if (id.header_flag == MIU) {
                    
                    Serial.print("Miu stream : "); Serial.println(can_gut.floats[0]);
                    Serial.print("From id : "); Serial.println(id.sender);
                }
                break;                    
            default:
                break;
                // do something
        }
        inner_frame->wrapped.internal_msg[0] = ACK; // read flag
    }
    if (stream_check_u && *stream_en ) {
        *stream_en = false;
        float u = curr_data->u;
        //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag)    
        send_msg(inner_frame, encodeCanId(myId, stream_id, STREAM, MIU), &u, sizeof(float));
        Serial.println("Sent stream u... ");
    }
    if (stream_check_y && *stream_en ) {
        *stream_en = false;
        float y = curr_data->out;
        //msg_to_can* inner_frame, uint16_t canm_id, void* data = nullptr ,size_t size_data = 0
        //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag);
        send_msg(inner_frame, encodeCanId(myId, stream_id, STREAM, LUX), &y, sizeof(float));
        Serial.println("Sent stream y... ");
    }
    return recv;
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

        uint64_t err_time = time_us_64();

        can->clearRXnOVRFlags(); 
        can->clearInterrupts();
        /*
        while (time_us_64() - err_time < 250) { 
            //wait on everything except control seq...
        }
        */
    }
    // else {
    //     //inner_frame->wrapped.internal_msg[3] - > post process errors
    //     //inner_frame->wrapped.internal_msg[2] - > post process interrupts
    //     can->clearRXnOVRFlags(); 
    //     can->clearInterrupts();
    // }    
}

void canbus_comm::ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd, void* val, size_t bytes) {
    if (bytes) {
        memcpy(&can_gut.floats[0], val, bytes);
        //memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
        can_gut.bytes[4] = req_cmd;
        //memcpy(inner_frame->wrapped.can_msg.data, &can_gut, sizeof(can_gut) - sizeof(uint8_t) - bytes);
        uint16_t can_id = encodeCanId(myId, req_id, SER_COM, SETS);    
        send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut)); // - sizeof(uint8_t) - bytes);
    }
    else {
        memcpy(&can_gut.bytes[0], &req_cmd, sizeof(uint8_t));
        //memcpy(inner_frame->wrapped.can_msg.data, &can_gut, sizeof(uint8_t));
        //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
        uint16_t can_id = encodeCanId(myId, req_id, SER_COM, GETS);    
        //msg_to_can* inner_frame , uint16_t id, void* data, 
        send_msg(inner_frame, can_id, &req_cmd, sizeof(uint8_t));
    }
}

void canbus_comm::ntwrk_calibration(msg_to_can* inner_frame) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    // float u[2] = {0.8, 0.2};
    float u[2] = {0.2, 0.8};
    float lux[2] = {0.0, 0.0};
    // float lux{0};
    float dist = luxmeter(get_ldr_voltage(LDR_PIN));
    int n_ack{0};
    uint32_t curr_time{0};
    uint16_t can_id;

    Serial.print("Getting Background Disturbance...");Serial.println(dist);
    can_id = encodeCanId(myId, BROADCAST, CALIBRATION, ACK);
    send_msg(inner_frame, can_id);
    curr_time = time_us_64()/1000;
    while(n_ack < NUM_NODES - 1 && (time_us_64()/1000 - curr_time) < THIRTY_SEC ) {
        recv_msg(inner_frame);
        if(process_msg_core0(inner_frame)){
            n_ack = (id.header_flag == ACK) ? ++n_ack : n_ack;
            Serial.print("Got confirmation end of read from ...");Serial.println(id.sender,HEX);
        }       
    }
    
    Serial.println("Begin Calibration Seq...");
    Serial.printf("%d nodes to calibrate\n", NUM_NODES);
    for (int n = 1; n <= NUM_NODES; n++) {
        if (myId == n - 1) {
            //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
            can_id = encodeCanId(myId, BROADCAST, CALIBRATION, START);
            //msg_to_can* inner_frame, uint16_t id, void* data=null, size_t = 0
            send_msg(inner_frame, can_id); //find way to send null data 
            for (int k = 0; k < 2; k++){
                Serial.println("Calibrating myId...");
/*------------------------------- CALIBRATION SEQ ------------------------------------------------*/
                analogWrite(LED_PIN, static_cast<int>(u[k]*(DAC_RANGE-1)));
                curr_time = time_us_64()/1000;
                int cnt{0};
                while ((time_us_64()/1000 - curr_time) < TEN_SEC) {
                    lux[k] += (luxmeter(get_ldr_voltage(LDR_PIN)));   
                    cnt++;                 
                }
                lux[k] /= cnt; 
                Serial.print("Done Measuring...");Serial.println(lux[k]);
                n_ack = 0;
                curr_time = time_us_64()/1000;
                while(n_ack < NUM_NODES - 1 && (time_us_64()/1000 - curr_time) < THIRTY_SEC ) {
                    recv_msg(inner_frame);
                    if( process_msg_core0(inner_frame)){
                        n_ack = (id.header_flag == ACK) ? ++n_ack : n_ack;
                        Serial.print("Got confirmation end of read from ...");Serial.println(id.sender,HEX);
                    }
                }
                can_id = encodeCanId(myId, BROADCAST, CALIBRATION, ACK);
                send_msg(inner_frame, can_id);
            }
            cxgains[myId] = (lux[1]-lux[0])/(u[1]-u[0]);
            Serial.printf("Gain %d->%d = %f\n", n-1, myId, cxgains[n-1]);
            //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
            can_id = encodeCanId(myId, BROADCAST, CALIBRATION, END);
            //msg_to_can* inner_frame, uint16_t id, void* data=null, size_t = 0
            send_msg(inner_frame, can_id); //find way to send null data 
            analogWrite(LED_PIN, static_cast<int>(0*(DAC_RANGE-1)));
        }
/*-------------------------------------------------------------------------------------------------*/            
        else {
            Serial.println("Waiting for a start ...");
            curr_time = time_us_64()/1000;
            while((time_us_64()/1000 - curr_time) < THIRTY_SEC) { 
                recv_msg(inner_frame);
                if(process_msg_core0(inner_frame)){
                    Serial.print("Message from ...");Serial.println(id.sender,HEX);
                    if (id.header_flag == START && id.sender == n - 1 ) break;
                }
            }
            
            Serial.println("Start received ...");
            for(int k = 0; k < 2; k++){
                curr_time = time_us_64()/1000;
                int cnt{0};
                while ((time_us_64()/1000 - curr_time) < TEN_SEC) {
                    lux[k] += (luxmeter(get_ldr_voltage(LDR_PIN))); 
                    cnt++;                   
                }
                lux[k] /= cnt;
                Serial.print("Done Measuring...");Serial.println(lux[k]);
                can_id = encodeCanId(myId, n - 1, CALIBRATION, ACK);
                send_msg(inner_frame, can_id);
                curr_time = time_us_64()/1000;
                while((time_us_64()/1000 - curr_time) < THIRTY_SEC) { 
                    recv_msg(inner_frame);
                    if(process_msg_core0(inner_frame)){
                        if (id.header_flag == ACK && id.sender == n - 1 ) break;
                    }
                }
            }
            cxgains[n-1] = (lux[1]-lux[0])/(u[1]-u[0]);
            Serial.printf("Gain %d->%d = %f\n", n-1, myId, cxgains[n-1]);
            Serial.println("Cross calibration end ...");
            curr_time = time_us_64()/1000;
            while((time_us_64()/1000 - curr_time) < THIRTY_SEC) { 
                recv_msg(inner_frame);
                if(process_msg_core0(inner_frame)){
                    if (id.header_flag == END && id.sender == n - 1 ) break;
                }
            }
        }
    } 
    Serial.print("Calibration end!");
//     int sign{1}; // 
//     for (int k : u) {
//         for (int n = 0; n < NUM_NODES; n++) {
//             if (myId == n) {
//                 Serial.println("Calibrating myId...");
//                 //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
//                 can_id = encodeCanId(myId, BROADCAST, CALIBRATION, START);
//                 //msg_to_can* inner_frame, uint16_t id, void* data=null, size_t = 0
//                 send_msg(inner_frame, can_id); //find way to send null data 
//                 int k_ack{0};
//                 curr_time = time_us_64()/1000;
//                 while(k < NUM_NODES - 1 && (time_us_64()/1000 - curr_time) < THIRTY_SEC ) {
//                     recv_msg(inner_frame);
//                     if (process_msg_core0(inner_frame)) {
//                         //decodeCanId(inner_frame->wrapped.can_msg.can_id,id.sender,id.receiver,id.header,id.header_flag);
//                         k = (id.header_flag == ACK) ? ++k : k;
//                         Serial.print("Got confirmation to start cal ...");Serial.println(id.sender,HEX);
//                     }
//                 }
// /*------------------------------- CALIBRATION SEQ ------------------------------------------------*/
//                 analogWrite(LED_PIN, static_cast<int>(u[k]*(DAC_RANGE-1)));
//                 curr_time = time_us_64()/1000;
//                 while ((time_us_64()/1000 - curr_time) < TEN_SEC) {
//                     lux = luxmeter(get_ldr_voltage(LDR_PIN));                    
//                 } 
//                 Serial.print("Done Measuring...");Serial.println(lux);
//                 can_id = encodeCanId(myId,BROADCAST,CALIBRATION,END); // MAYBE CHANGE FROM REQ TO A DIFF FLAG
//                 send_msg(inner_frame, can_id);
//                 k_ack = 0;
//                 curr_time = time_us_64()/1000;
//                 while(k < NUM_NODES - 1 && (time_us_64()/1000 - curr_time) < THIRTY_SEC ) {
//                     recv_msg(inner_frame);
//                     if (process_msg_core0(inner_frame)) {
//                         //decodeCanId(inner_frame->wrapped.can_msg.can_id,id.sender,id.receiver,id.header,id.header_flag);
//                         k = (id.header_flag == ACK) ? ++k : k;
//                         Serial.print("Got confirmation end of read from ...");Serial.println(id.sender,HEX);
//                     }
//                 }
//                 Serial.println("Calibration over ...");
//                 analogWrite(LED_PIN, 0);
//                 cxgains[n] = cxgains[n] + sign * lux/(u[0]-u[1]);
//                 Serial.print("Updated own gain value "); Serial.println(cxgains[n]);
//                 // wait time to let the light die out
//                 curr_time = time_us_64()/1000;
//                 while((time_us_64()/1000 - curr_time) < FIVE_SEC) { 

//                 }
//             }
// /*-------------------------------------------------------------------------------------------------*/            
//             else {
//                 Serial.println("Waiting for a start ...");
//                 curr_time = time_us_64()/1000;
//                 while((time_us_64()/1000 - curr_time) < THIRTY_SEC) { 
//                     recv_msg(inner_frame);
//                     if(process_msg_core0(inner_frame)) {
//                         Serial.print("Message from ...");Serial.println(id.sender,HEX);
//                         if (id.header_flag == START && id.sender == n ) {
//                             can_id = encodeCanId(myId, n, CALIBRATION, ACK); // MAYBE CHANGE FROM REQ TO A DIFF FLAG
//                             send_msg(inner_frame,can_id);
//                         break;    
//                         }
//                     }
//                 }
//                 Serial.println("Request received ...");
//                 //process_msg_core0(inner_frame);
//                 curr_time = time_us_64()/1000;
//                 while (id.header_flag == START && (time_us_64()/1000 - curr_time) < THIRTY_SEC) {
//                     recv_msg(inner_frame);
//                     //cross calibration myId -> current node
//                     lux = luxmeter(get_ldr_voltage(LDR_PIN));
//                     if (process_msg_core0(inner_frame) && id.header_flag == END) { // ASSUMING THE MSG IS END...
//                         Serial.print("Got confirmation to stop cross gain reads from ...");Serial.println(id.sender,HEX);
//                         //uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag
//                         can_id = encodeCanId(myId, n, CALIBRATION, ACK);
//                         //msg_to_can* inner_frame, uint16_t id, void* data=null, size_t = 0
//                         send_msg(inner_frame, can_id);
//                     }
//                 }
//                 Serial.println("Cross calibration end ...");
//                 cxgains[n] = cxgains[n] + sign * lux/(u[0]-u[1]);
//                 Serial.print("Updated cross gain value "); Serial.println(cxgains[n]);
//                 Serial.print("With... "); Serial.println(lux/(u[0] - u[1]));
//                 // wait time to let the light die out
//                 curr_time = time_us_64()/1000;
//                 while((time_us_64()/1000 - curr_time) < FIVE_SEC) { 

//                 }
//             }
//         }
//         sign = -1;
//     } 
//     Serial.print("Calibration end!");
}

//decode outside, only go to ser recv 
void canbus_comm::ser_reply(msg_to_can* inner_frame ,data_reads* curr_data, ser_data* info) {
    //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag);
    uint16_t can_id = encodeCanId(id.receiver ,id.sender, SER_COM, ACK);

    if (id.header_flag == SETS) {
        Serial.println("Got a SET req...");
        switch (can_gut.bytes[4]) {
            // Sets
            case set_ref:
                Serial.println("set_ref request from :" ); Serial.print(id.sender);
                // handle set_ref
                PID.set_reference(can_gut.floats[0]);
                break;
            case set_u:
                Serial.println("set_u request from :" ); Serial.print(id.sender);
                // handle set_u
                set_dutycycle(can_gut.floats[0]);
                break;
            case set_occ: {
                Serial.println("set_occ request from :" ); Serial.print(id.sender);
                // handle set_occ
                occ_st = occ_st ^ true;
                float reference = (occ_st == true) ? PID.r_h : PID.r_l;
                PID.set_reference(reference); //ref);
                break;
            }
            case set_aa:
                Serial.println("set_aa request from :" ); Serial.print(id.sender);
                // handle set_occ
                PID.set_anti_wu_status(can_gut.bools[0]);
                break;            
            case set_fb:
                Serial.println("set_fb request from :" ); Serial.print(id.sender);
                // handle set_occ
                PID.set_fb_status(can_gut.bools[0]);
                break;  

            // Distribution
            case get_lower_bound_occ:
                Serial.println("get_lower_bound_occ request from :" ); Serial.print(id.sender);
                // handle get_lower_bound_occ
                break;
            case set_lower_bound_occ:
                Serial.println("set_lower_bound_occ request from :" ); Serial.print(id.sender);
                // handle set_lower_bound_occ
                break;
            case get_lower_bound_unocc:
                Serial.println("get_lower_bound_unocc request from :" ); Serial.print(id.sender);
                // handle get_lower_bound_unocc
                break;
            case set_lower_bound_unocc:
                Serial.println("set_lower_bound_unocc request from :" ); Serial.print(id.sender);
                // handle set_lower_bound_unocc
                break;
            case get_curr_cost:
                Serial.println("get_curr_cost request from :" ); Serial.print(id.sender);
                // handle get_curr_cost
                break;
            case get_curr_lum:
                Serial.println("get_curr_lum request from :" ); Serial.print(id.sender);
                // handle get_curr_lum
                break;
            case set_curr_cost:
                Serial.println("set_curr_cost request from :" ); Serial.print(id.sender);
                // handle set_curr_cost
                break;

            default:
                // handle unknown request
                Serial.println("unknown_command request from :" ); Serial.print(id.sender);
                break;          
       }
    }
    else if (id.header_flag == GETS) {
        // Gets
        uint8_t req_cmd{0};
        Serial.println("Got a GET req...");
        switch (can_gut.bytes[0]) {
            case get_ref:
                Serial.print("get_ref request from : ");Serial.println(id.sender);
                // handle get_ref
                {   
                    float refr = PID.get_reference(); //PID.get_disturbance(u);
                    req_cmd = get_ref;
                    memcpy(&can_gut.floats[0], &refr, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                } 
                break;
            case get_u:
                Serial.print("get_u request from : ");Serial.println(id.sender);
                // handle get_u
                //can_gut.floats[0] = u;
                {   
                    req_cmd = get_u;
                    float u = curr_data->u;
                    memcpy(&can_gut.floats[0], &u, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                } 
                break;
            case get_y:
                Serial.print("get_y request from : ");Serial.println(id.sender);
                // handle get_y
                //can_gut.floats[0] = L;
                {   
                    req_cmd = get_y;
                    float y = curr_data->out;
                    memcpy(&can_gut.floats[0], &y, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                
                break;
            case get_volt:
                Serial.print("get_volt request from : ");Serial.println(id.sender);
                // handle get_volt
                //can_gut.floats[0] = v;
                {   
                    req_cmd = get_volt;
                    memcpy(&can_gut.floats[0], &info->voltage, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                
                break;
            case get_occ:
                Serial.print("get_occ request from : ");Serial.println(id.sender);
                // handle get_occ

                {   
                    req_cmd = get_occ;
                    memcpy(&can_gut.bools[0], &occ_st, sizeof(bool));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                 
                break;
            case get_aa:
                Serial.print("get_aa request from : ");Serial.println(id.sender);
                // handle get_aa
                {   
                    bool aa_status = PID.get_anti_wu_status(); //PID.get_disturbance(u);
                    req_cmd = get_aa;
                    memcpy(&can_gut.bools[0], &aa_status, sizeof(bool));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                } 
                break;
            case get_fb:
                Serial.print("get_fb request from : ");Serial.println(id.sender);
                // handle get_fb
                {   
                    bool fb_status = PID.get_fb_status(); //PID.get_disturbance(u);
                    req_cmd = get_fb;
                    memcpy(&can_gut.bools[0], &fb_status, sizeof(bool));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                
                break;
            case get_dist:
                Serial.print("get_dist request from : ");Serial.println(id.sender);
                // handle get_dist
                {   
                    float dist = PID.get_disturbance(curr_data->u);
                    req_cmd = get_dist;
                    memcpy(&can_gut.floats[0], &dist, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_pwr:
                Serial.print("get_pwr request from : ");Serial.println(id.sender);
                // handle get_pwr
                {   
                    float pwr = inst_power_consumption(uk_1);
                    req_cmd = get_pwr;
                    memcpy(&can_gut.floats[0], &pwr, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_Rtime:
                Serial.print("get_Rtime request from : ");Serial.println(id.sender);
                // handle get_Rtime
                {           
                    uint32_t elapsed_time = time_us_64()/1000 - last_restart;
                    req_cmd = get_Rtime;
                    memcpy(&can_gut.four_bytes[0], &elapsed_time, sizeof(uint32_t));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));        
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            // Performance metrics
            case get_flicker:
                Serial.print("get_flicker request from : ");Serial.println(id.sender);
                // handle get_flicker
                {   
                    float flicker_avg = info->flicker/info->N; //flicker/N;
                    req_cmd = get_flicker;
                    memcpy(&can_gut.floats[0], &flicker_avg, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_vis:
                Serial.print("get_vis request from : ");Serial.println(id.sender);
                // handle get_vis
                {   
                    float vis_avg = info->visibility/info->N; //vis/N;
                    req_cmd = get_vis;
                    memcpy(&can_gut.floats[0], &vis_avg, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_energy:
                Serial.print("get_energy request from : ");Serial.println(id.sender);
                // handle get_energy
                {
                    req_cmd = get_energy;
                    memcpy(&can_gut.floats[0], &info->energy, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_buff_u:
                Serial.print("get_buff_u request from : ");Serial.println(id.sender);
                // handle get_buff_u
                //to do
                break;
            case get_buff_y:
                Serial.print("get_buff_y request from : ");Serial.println(id.sender);
                // handle get_buff_y
                //to do
                break;
            // Streams
            case start_stream_u:
                Serial.println("start_stream_u request from :" ); Serial.print(id.sender);
                stream_check_u = true;
                stream_id = id.sender;
                //to - do
                break;
            case start_stream_y:
                Serial.println("start_stream_y request from :" ); Serial.print(id.sender);
                stream_check_y = true;
                stream_id = id.sender;
                break;
            case stop_stream_u:
                Serial.println("stop_stream_u request from :" ); Serial.print(id.sender);
                stream_check_u = false;
                stream_id = 0xff;
                break;
            case stop_stream_y:
                Serial.println("stop_stream_y request from :" ); Serial.print(id.sender);
                stream_check_y = false;
                stream_id = 0xff;
                break;
            // Reset
            case reset:
                Serial.println("reset");
                // handle reset
                occ_st = UNOCC;
                N = 0;
                ener = 0;
                vis_err = 0;
                flicker = 0;
                //_data_log->clear();
                last_restart = time_us_64()/1000;
                adjust_gain();
                PID.set_reference(occ_st ? PID.r_h : PID.r_l);
                PID.set_system_gain_n_dist(G,d);
                //init boot
                //init network
                break;
            default:
                Serial.print("unknown request from : "); Serial.println(id.sender);
                // do something
                break;
            }
    }
}


void canbus_comm::ser_receive(msg_to_can* inner_frame) {

    switch (can_gut.bytes[4]) {
        // Gets
        case get_ref:
            Serial.print("Ref : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_u:
            Serial.print("PWM : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_y:
            Serial.print("Lux : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_volt:
            Serial.print("Voltage : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_occ:
            Serial.print("Occupancy : "); Serial.println(can_gut.bools[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_aa:
            Serial.print("Anti-Windup : "); Serial.println(can_gut.bools[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_fb:
            Serial.print("Feed-Backward : "); Serial.println(can_gut.bools[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_dist:
            Serial.print("Disturbance : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_pwr:
            Serial.print("Power : "); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_Rtime:
            Serial.print("Time since restart : "); Serial.println(can_gut.four_bytes[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_buff_u: 
            {
                // to do
            }
            break;
        case get_buff_y:
            {

                // to do
            }
            break;

        // Performance metrics
        case get_flicker:
            Serial.print("Flicker :"); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_vis:
            Serial.print("Visibility :"); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;
        case get_energy:
            Serial.print("Energy :"); Serial.println(can_gut.floats[0]);
            Serial.print("From id : "); Serial.println(id.sender);
            break;

        // Distribution
        case get_lower_bound_occ:
            //to - do
            break;
        case set_lower_bound_occ:
            //to - do
            break;
        case get_lower_bound_unocc:
            //to - do
            break;
        case set_lower_bound_unocc:
            //to - do
            break;
        case get_curr_cost:
            //to - do
            break;
        case get_curr_lum:
            //to - do
            break;
        default:
            Serial.print("err on reply : "); Serial.print(can_gut.bytes[4]);
            break;
      }
}


