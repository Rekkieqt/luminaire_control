#include <Arduino.h>
#include "performance.h"
#include "comms.h"
#include "init.h"
#include "boot.h"
#include "ldr.h"
#include "pid.h"
#include "command.h"

canbus_comm::canbus_comm() {
    assign_cross_gain_vector();
}

canbus_comm::~canbus_comm() {
    delete[] cxgains;
}

can_data_decoder canbus_comm::can_gut;

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
        //Serial.println("Caught inner frame!"); 
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

bool canbus_comm::process_msg_core0(msg_to_can* inner_frame, bool *stream_en) { //receive can thru fifo, process data, do something on necessity
    bool recv{false};
    if (inner_frame->wrapped.internal_msg[0] == REQUEST) {
        recv = true;
        Serial.println("Can Msg Received!");
        //dest, source, size
        //memcpy(&can_data,inner_frame->wrapped.can_msg.data,sizeof(can_data));
        memcpy(&can_gut,inner_frame->wrapped.can_msg.data,inner_frame->wrapped.can_msg.can_dlc);
        // print
        /*---------------------------------------- decode id ----------------------------------------*/
        decodeCanId(inner_frame->wrapped.can_msg.can_id, id.sender, id.receiver, id.header, id.header_flag);
        /*-------------------------------------------------------------------------------------------*/
        Serial.print("Sender Id ");Serial.println(id.sender,HEX);
        Serial.print("Receiver Id ");Serial.println(id.receiver,HEX);
        Serial.print("Header ");Serial.println(id.header,HEX);
        Serial.print("Header flag ");Serial.println(id.header_flag,HEX);
        Serial.print("Msg dlc ");Serial.println(inner_frame->wrapped.can_msg.can_dlc);
        // Serial.print("Gut Size ");Serial.println(sizeof(can_gut));
        // //Serial.print("Data ");Serial.println(can_gut.eight_bytes);
        // Serial.print("Floats [0] ");Serial.println(can_gut.floats[0]);
        // Serial.print("Floats [1] ");Serial.println(can_gut.floats[1]);
        // Serial.print("Type of Request [0] ");Serial.println(can_gut.bytes[0]);
        // Serial.print("Type of Request [4] ");Serial.println(can_gut.bytes[4]);
        // Serial.print("Type of Request [7] ");Serial.println(can_gut.bytes[0]);
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
                    ser_reply(inner_frame);
                }
                else if (id.header_flag == ACK) {
                    ser_receive(inner_frame);
                }
                // do something
                break;
            case REFERENCE:
                // do something
                break;
            case RESTART:
                // do something
                break; 
            case STREAM:
                //memcpy(&can_gut.floats[0],inner_frame->wrapped.can_msg.data,inner_frame->wrapped.can_msg.can_dlc);
                if(id.header_flag == LUX){
                    
                    Serial.print("Lux stream :"); Serial.println(can_gut.floats[0]);
                    Serial.print("From id : "); Serial.println(id.sender);
                }
                else if (id.header_flag == MIU) {
                    
                    Serial.print("Miu stream :"); Serial.println(can_gut.floats[0]);
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
        float pwm{54};
        *stream_en = false;
        //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag)    
        send_msg(inner_frame, encodeCanId(myId,stream_id,STREAM,MIU), &pwm, sizeof(float));
        Serial.println("Sent stream u... ");
    }
    if (stream_check_y && *stream_en ) {
        float out_lux{19};
        *stream_en = false;
        //msg_to_can* inner_frame, uint16_t canm_id, void* data = nullptr ,size_t size_data = 0
        //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag);
        send_msg(inner_frame, encodeCanId(myId,stream_id,STREAM,LUX), &out_lux, sizeof(float));
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
        can_gut.bytes[7] = req_cmd;
        //memcpy(inner_frame->wrapped.can_msg.data, &can_gut, sizeof(can_gut) - sizeof(uint8_t) - bytes);
        uint16_t can_id = encodeCanId(myId, req_id, SER_COM, SETS);    
        send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut)); // - sizeof(uint8_t) - bytes);
    }
    else {
        memcpy(&can_gut.bytes[7], &req_cmd, sizeof(uint8_t));
        //memcpy(inner_frame->wrapped.can_msg.data, &can_gut, sizeof(uint8_t));
        //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
        uint16_t can_id = encodeCanId(myId, req_id, SER_COM, GETS);    
        //msg_to_can* inner_frame , uint16_t id, void* data, 
        send_msg(inner_frame, can_id, &req_cmd, sizeof(uint8_t));
    }
}

void canbus_comm::assign_cross_gain_vector() {
    cxgains = new float[NUM_NODES];
    memset(cxgains,0 ,NUM_NODES*sizeof(float));
}

void canbus_comm::set_ntwrk_params(uint8_t id, uint8_t node_max){
    myId = id;
    NUM_NODES = node_max;
}

void canbus_comm::ntwrk_calibration(msg_to_can* inner_frame) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    float u[2] = {0.2, 0.8};
    float lux[2] = {0.0, 0.0};
    float dist = luxmeter(get_ldr_voltage(LDR_PIN));
    int n_ack{0};
    uint32_t curr_time{0};
    uint16_t can_id;

    Serial.print("Getting Background Disturbance...");Serial.println(dist);
    can_id = encodeCanId(myId, BROADCAST, CALIBRATION, ACK);
    send_msg(inner_frame, can_id);
    curr_time = time_us_64()/1000;
    while(n_ack < NUM_NODES - 1 && (time_us_64()/1000 - curr_time) < THIRTY_SEC ) {
        if(recv_msg(inner_frame)){
            process_msg_core0(inner_frame);
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
            for(int k = 0; k < 2; k++){
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
                    if(recv_msg(inner_frame)){
                        process_msg_core0(inner_frame);
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
                if(recv_msg(inner_frame)){
                    process_msg_core0(inner_frame);
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
                    if(recv_msg(inner_frame)){
                        process_msg_core0(inner_frame);
                        if (id.header_flag == ACK && id.sender == n - 1 ) break;
                    }
                }
            }
            cxgains[n-1] = (lux[1]-lux[0])/(u[1]-u[0]);
            Serial.printf("Gain %d->%d = %f\n", n-1, myId, cxgains[n-1]);
            Serial.println("Cross calibration end ...");
            curr_time = time_us_64()/1000;
            while((time_us_64()/1000 - curr_time) < THIRTY_SEC) { 
                if(recv_msg(inner_frame)){
                    process_msg_core0(inner_frame);
                    if (id.header_flag == END && id.sender == n - 1 ) break;
                }
            }
        }
    } 
    Serial.print("Calibration end!");
//     for (int k : u) {
//         for (int n = 1; n <= NUM_NODES; n++) {
//             if (myId == n) {
//                 Serial.println("Calibrating myId...");
//                 //uint8_t sender, uint8_t receiver, uint8_t task, uint8_t flags
//                 can_id = encodeCanId(myId, BROADCAST, CALIBRATION, START);
//                 //msg_to_can* inner_frame, uint16_t id, void* data=null, size_t = 0
//                 send_msg(inner_frame, can_id); //find way to send null data 
// /*------------------------------- CALIBRATION SEQ ------------------------------------------------*/
//                 analogWrite(LED_PIN, static_cast<int>(u[k]*(DAC_RANGE-1)));
//                 curr_time = time_us_64()/1000;
//                 while ((time_us_64()/1000 - curr_time) < TEN_SEC) {
//                     lux = luxmeter(get_ldr_voltage(LDR_PIN));                    
//                 } 
//                 Serial.print("Done Measuring...");Serial.println(lux);
//                 can_id = encodeCanId(myId,BROADCAST,CALIBRATION,END); // MAYBE CHANGE FROM REQ TO A DIFF FLAG
//                 send_msg(inner_frame, can_id);
//                 int k{0};
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
//                 cxgains[n - 1] -= lux/(u[0]-u[1]);
//                 Serial.print("Updated own gain value "); Serial.println(cxgains[n - 1]);
//                 Serial.print("With... "); Serial.println(lux/(u[1]-u[2]));
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
//                         if (id.header_flag == START && id.sender == n ) break;
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
//                 cxgains[n - 1] -= lux/(u[1]-u[2]);
//                 Serial.print("Updated cross gain value "); Serial.println(cxgains[n - 1]);
//                 Serial.print("With... "); Serial.println(lux/(u[1]-u[2]));
//                 // wait time to let the light die out
//                 curr_time = time_us_64()/1000;
//                 while((time_us_64()/1000 - curr_time) < FIVE_SEC) { 

//                 }
//             }
//         }
//     } 
//     Serial.print("Calibration end!");
}

//decode outside, only go to ser recv 
void canbus_comm::ser_reply(msg_to_can* inner_frame) {
    //encodeCanId(uint8_t sender, uint8_t receiver, uint8_t header, uint8_t header_flag);
    uint16_t can_id = encodeCanId(id.receiver ,id.sender, SER_COM, ACK);

    if (id.header_flag == SETS) {
        Serial.println("Got a SET req...");
        switch (can_gut.bytes[7]) {
            // Sets
            case set_ref:
                Serial.println("SET: set_ref");
                // handle set_ref
                PID.set_reference(can_gut.floats[0]);
                break;
            case set_u:
                Serial.println("SET: set_u");
                // handle set_u
                set_dutycycle(can_gut.floats[0]);
                break;
            case set_occ:
                Serial.println("SET: set_occ");
                // handle set_occ
                occ_st = occ_st ^ true;
                //ref = (occ_st == true) ? PID.r_h : PID.r_l;
                PID.set_reference(0.0f); //ref);
                break;
            case set_aa:
                Serial.println("SET: set_aa");
                // handle set_occ
                PID.set_anti_wu_status(can_gut.bools[0]);
                break;            
            case set_fb:
                Serial.println("SET: set_fb");
                // handle set_occ
                PID.set_fb_status(can_gut.bools[0]);
                break;  

            // Distribution
            case get_lower_bound_occ:
                Serial.println("SET: get_lower_bound_occ");
                // handle get_lower_bound_occ
                break;
            case set_lower_bound_occ:
                Serial.println("SET: set_lower_bound_occ");
                // handle set_lower_bound_occ
                break;
            case get_lower_bound_unocc:
                Serial.println("SET: get_lower_bound_unocc");
                // handle get_lower_bound_unocc
                break;
            case set_lower_bound_unocc:
                Serial.println("SET: set_lower_bound_unocc");
                // handle set_lower_bound_unocc
                break;
            case get_curr_cost:
                Serial.println("SET: get_curr_cost");
                // handle get_curr_cost
                break;
            case get_curr_lum:
                Serial.println("SET: get_curr_lum");
                // handle get_curr_lum
                break;
            case set_curr_cost:
                Serial.println("SET: set_curr_cost");
                // handle set_curr_cost
                break;

            default:
                // handle unknown request
                Serial.println("SET: unknown_command");
                break;          
       }
    }
    else if (id.header_flag == GETS) {
        // Gets
        uint8_t req_cmd{0};
        Serial.println("Got a GET req...");
        switch (can_gut.bytes[7]) {
            case get_ref:
                Serial.println("GET: get_ref");
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
                Serial.println("GET: get_u");
                // handle get_u
                //can_gut.floats[0] = u;
                {   
                    float u = 32; //PID.get_disturbance(u);
                    float u_extra = 64;
                    req_cmd = get_u;
                    memcpy(&can_gut.floats[0], &u, sizeof(float));
                    memcpy(&can_gut.floats[1], &u_extra, sizeof(float));
                    //memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                } 
                break;
            case get_y:
                Serial.println("GET: get_y");
                // handle get_y
                //can_gut.floats[0] = L;
                {   
                    float y = 13.8; //PID.get_disturbance(u);
                    req_cmd = get_dist;
                    memcpy(&can_gut.floats[0], &y, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                
                break;
            case get_volt:
                Serial.println("GET: get_volt");
                // handle get_volt
                //can_gut.floats[0] = v;
                {   
                    float volt = 33.9; //PID.get_disturbance(u);
                    req_cmd = get_volt;
                    memcpy(&can_gut.floats[0], &volt, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }                
                break;
            case get_occ:
                Serial.println("GET: get_occ");
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
                Serial.println("GET: get_aa");
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
                Serial.println("GET: get_fb");
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
                Serial.println("GET: get_dist");
                // handle get_dist
                //can_gut.floats[0] = PID.get_disturbance(u);
                {   
                    float dist = 4.9; //PID.get_disturbance(u);
                    req_cmd = get_dist;
                    memcpy(&can_gut.floats[0], &dist, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_pwr:
                Serial.println("GET: get_pwr");
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
                Serial.println("GET: get_Rtime");
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
            case get_buff_u:
                Serial.println("GET: get_buff_u");
                // handle get_buff_u
                //to do
                break;
            case get_buff_y:
                Serial.println("GET: get_buff_y");
                // handle get_buff_y
                //to do
                break;
            // Streams
            case start_stream_u:
                Serial.println("SET: start_stream_u");
                stream_check_u = true;
                stream_id = id.sender;
                //to - do
                break;
            case start_stream_y:
                Serial.println("SET: start_stream_y");
                stream_check_y = true;
                stream_id = id.sender;
                break;
            case stop_stream_u:
                Serial.println("SET: stop_stream_u");
                stream_check_u = false;
                stream_id = 0xff;
                break;
            case stop_stream_y:
                Serial.println("SET: stop_stream_y");
                stream_check_y = false;
                stream_id = 0xff;
                break;
            // Performance metrics
            case get_flicker:
                Serial.println("GET: get_flicker");
                // handle get_flicker
                {   
                    float flicker_avg = 44; //flicker/N;
                    req_cmd = get_flicker;
                    memcpy(&can_gut.floats[0], &flicker_avg, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_vis:
                Serial.println("GET: get_vis");
                // handle get_vis
                {   
                    float vis_avg = 69; //vis/N;
                    req_cmd = get_vis;
                    memcpy(&can_gut.floats[0], &vis_avg, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
                break;
            case get_energy:
                Serial.println("GET: get_energy");
                // handle get_energy
                {
                    float ener = 1337;
                    req_cmd = get_energy;
                    memcpy(&can_gut.floats[0], &ener, sizeof(float));
                    memcpy(&can_gut.bytes[4], &req_cmd, sizeof(uint8_t));
                    can_id = encodeCanId(myId ,id.sender, SER_COM, ACK);
                    send_msg(inner_frame, can_id, &can_gut, sizeof(can_gut));
                }
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
                Serial.println("GET: unknown request");
                // do something
                break;
            }
    }
    memset(&can_gut,0 , sizeof(can_gut));
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
      memset(&can_gut,0,sizeof(can_gut));
}


