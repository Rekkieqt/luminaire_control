#include <Arduino.h>
#include "luxmeter.h"
#include "comms.h"
#include "ring_buffer.h"
#include "init.h"

serial_comm::serial_comm() {

}

serial_comm::~serial_comm() {
}

void serial_comm::func_test(int msg1, float msg2) {
    Serial.println("Working msg detectin"); 
}

void serial_comm::decode(void* user_input, int n_bytes, luxmeter &meas, data_reads &curr_data, ring_buffer &curr_buff) {
    if (n_bytes/4 == MSG_SIZE && n_bytes > 0) {
        memcpy(decoder.input_ascii, user_input, n_bytes);    
    }
    else {
        // wrong buffer size, prevent buffer overflow
        Serial.println("Wrong message size!"); 
    }

  switch(decoder.input_ascii[0]) {
  //BASIC COMMANDS
        case duty_msg :
            //Set directly the duty cycle
            func_test(decoder.input_ascii[0], decoder.input_value[1]);
            break;
        case get_msg:
              switch(decoder.input_ascii[0]) {                    
//BASIC COMMANDS          
                    case duty_msg:
                        //Get current duty cycle
                        func_test(decoder.input_ascii[0]);
                        break;
                    case ref_msg:
                        //Get current illuminance reference of luminaire i
                        func_test(decoder.input_ascii[0]);
                        break;
                    case lux_msg:
                        // Measure the actual illuminance (LUX sensor) of luminaire i
                        func_test(decoder.input_ascii[0]);
                        break;
                    case volt_msg:
                        ////Measure the voltage level at the LDR at luminaire i
                        func_test(decoder.input_ascii[0]);
                        break;
                    case occupancy_msg:
                        //Get the current occupancy state of desk <i>
                        func_test(decoder.input_ascii[0]);
                        break;
                    case a_msg:
                        //Get anti-windup state of desk <i>
                        func_test(decoder.input_ascii[0]);
                        break;
                    case f_msg:
                        //Get feedback control state of desk <i>                    
                        func_test(decoder.input_ascii[0]);
                        break;
                    case dist_msg:
                        //Get current external illuminance of desk <i>
                        func_test(decoder.input_ascii[0]);
                        break;
                    case p_msg:
                        //Get instantaneous power consumption of desk <i>
                        func_test(decoder.input_ascii[0]);
                        break;
                    case time_msg:
                        //Get the elapsed time since the last restart
                        func_test(decoder.input_ascii[0]);
                        break;
                    case b_msg:
                        //Get the last minute buffer of the variable <x> of the desk <i>. <x> can be 'y' or 'u'.
                        func_test(decoder.input_ascii[0], decoder.input_value[1]);
                        break;
                    //PERFORMANCE COMMANDS
                    case E_msg:
                        //Get the average energy consumption at the desk <i> since the last system restart.
                        func_test(decoder.input_ascii[0]);
                        break;
                    case V_msg:
                        //Get the average visibility error at desk <i> since the last system restart.
                        func_test(decoder.input_ascii[0]);
                        break;
                    case F_msg:
                        //Get the average flicker error on desk <i> since the last system restart.
                        func_test(decoder.input_ascii[0]);
                        break;
                    default:
                        Serial.println ("Invalid command");
                        break;
    }
            break;
        case ref_msg:
            //Set the illuminance reference of ill i
            func_test(decoder.input_ascii[0], decoder.input_value[1]);
            break;
        case occupancy_msg:
            //Set the current occupancy state of desk <i>
            func_test(decoder.input_ascii[0], decoder.input_value[1]);
            break;
        case f_msg:
            //Set feedback control on/off on desk <i>        
            func_test(decoder.input_ascii[0], decoder.input_value[1]);
            break;
        case a_msg:
            //Set anti-windup on/off on at desk <i>
            func_test(decoder.input_ascii[0], decoder.input_value[1]);
            break;            
        case data_stream_start_msg:
            //Start the stream of the real-time variable <x> of desk <i>. <x> can be 'y' or 'u'.
            if (decoder.input_value[1] == lux_msg) {
                func_test(decoder.input_ascii[0], decoder.input_value[1]);
            }
            else if(decoder.input_value[1] == x_msg){
                func_test(decoder.input_ascii[0], decoder.input_value[1]);    
            }
            else {
                Serial.println ("Invalid command");
            }
            break;
        case data_stream_stop_msg:
            //Stop the stream of the real-time variable <x> of desk <i>. <x> can be 'y' or 'u'
                        if (decoder.input_value[1] == lux_msg) {
                func_test(decoder.input_ascii[0], decoder.input_value[1]);
            }
            else if(decoder.input_value[1] == x_msg){
                func_test(decoder.input_ascii[0], decoder.input_value[1]);    
            }
            else {
                Serial.println ("Invalid command");
            }
            break;
        default:
            Serial.println ("Invalid command");
            break;
    }
}

canbus_comm::canbus_comm() {

}

canbus_comm::~canbus_comm() {
}

void canbus_comm::inner_frm_to_fifo(msg_to_can* inner_frame) {
    for(int i = 0; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) { 
        rp2040.fifo.push_nb(inner_frame->in_msg[i]);
    }
}

void canbus_comm::recv_msg(msg_to_can* inner_frame) {
    if (rp2040.fifo.pop_nb(&inner_frame->in_msg[0])) {
        for (int i = 1; i < sizeof(inner_frame->in_msg) / sizeof(uint32_t); i++) {
        rp2040.fifo.pop_nb(&inner_frame->in_msg[i]);
        }   
    }
}

void canbus_comm::send_can(msg_to_can* inner_frame, MCP2515* can) {
    if (inner_frame->wrapped.internal_msg[0] == 0xff){
        inner_frame->wrapped.internal_msg[0] = 0x00;
        can->sendMessage(&inner_frame->wrapped.can_msg);            
        inner_frame->wrapped.internal_msg[1] = ERR_INFO; //special flag for just error frm
        inner_frame->wrapped.internal_msg[2] = can->getInterrupts();
        inner_frame->wrapped.internal_msg[3] = can->getErrorFlags();
        inner_frm_to_fifo(inner_frame);
    }
}

void canbus_comm::send_msg(uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame) {
    inner_frame->wrapped.can_msg.can_id = id;
    inner_frame->wrapped.can_msg.can_dlc = sizeof(data);
    memcpy(inner_frame->wrapped.can_msg.data, &data, sizeof(data));
    inner_frame->wrapped.internal_msg[0] = 0xff; // unread flag
    inner_frame->wrapped.internal_msg[1] = header;
    inner_frame->wrapped.internal_msg[2] = sizeof(can_frame);
    //inner_frame.internal_msg[2] = 0; extra 
    inner_frm_to_fifo(inner_frame);
}

void canbus_comm::process_can_core1(msg_to_can* inner_frame, MCP2515* can, volatile bool& _got_irq) { //receive can bus, send to core 0 through fifo, maybe do sum if necessary
    if (_got_irq) {
        _got_irq = false;
        inner_frame->wrapped.internal_msg[0] = 0xff; // unread flag
        inner_frame->wrapped.internal_msg[1] = 0x00; // inner frame header 
        inner_frame->wrapped.internal_msg[2] = can->getInterrupts(); //irq
        inner_frame->wrapped.internal_msg[3] = can->getErrorFlags(); //errors
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX0IF) {
            can->readMessage( MCP2515::RXB0, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            }
        if(inner_frame->wrapped.internal_msg[2] & MCP2515::CANINTF_RX1IF) {
            can->readMessage( MCP2515::RXB1, &inner_frame->wrapped.can_msg );
            inner_frm_to_fifo(inner_frame);
            }
        if(inner_frame->wrapped.internal_msg[3] & 0b11111000) {
            //maybe something extra
            can->clearRXnOVRFlags(); 
            can->clearInterrupts();
        }
    } 
}

void canbus_comm::process_msg_core0(msg_to_can* inner_frame) { //receive can thru fifo, process data, do something on necessity
    if(inner_frame->wrapped.internal_msg[0] == 0xff) {
        inner_frame->wrapped.internal_msg[0] = 0x00; //mark as read
        switch (inner_frame->wrapped.internal_msg[1]) {
          case BROADCAST:
            // print
            Serial.println("Msg Broadcast!");
            break;
          case ERR_INFO: {
            char canintf_str[] {"| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
            char eflg_str [] {"| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | "};
            //if x inner_frame->internal_msg[1];
            Serial.println("-----------------------------------------------------------------");
            Serial.println( canintf_str );
            Serial.print(" | ");
            for (int bit = 7; bit >= 0; bit--) {
                Serial.print(" "); Serial.write( bitRead(inner_frame->wrapped.internal_msg[1], bit ) ? '1' : '0' ); Serial.print(" | ");
            }
            //if x inner_frame->internal_msg[2];
            Serial.println("");
            Serial.println("-----------------------------------------------------------------");
            Serial.println(eflg_str);
            Serial.print("| ");
            for (int bit = 7; bit >= 0; bit--) {
                Serial.print(" "); Serial.write(bitRead(inner_frame->wrapped.internal_msg[2], bit) ? '1' : '0'); Serial.print(" | ");
            }
            Serial.println("");
            Serial.println("-----------------------------------------------------------------");
            break;
        }
          default:
            // do something 
            Serial.println("Header Error!");
        }
    }
}