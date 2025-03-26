#include <Arduino.h>
#include "luxmeter.h"
#include "comms.h"
#include "ring_buffer.h"
#include "init.h"

serial_comm::serial_comm() {

}

serial_comm::~serial_comm() {
}

String serial_comm::cut_string(String ser_msg, int start_point, int n_chars){
    return ser_msg.substring(start_point, start_point + n_chars);
} 

int serial_comm::get_int_from_str(String ser_msg) {
    return atoi(ser_msg.c_str());
}

bool serial_comm::decode(msg_read* user_input, size_t n_bytes, luxmeter* lmeas, data_reads* curr_data,
                         ring_buffer* curr_buff, static_lux_data* lux_info, user_set_flags* usr_flags, perf_meas* perf_data, 
                         int& ref , int& pwm) {
    if (n_bytes > MSG_SIZE - 1) {
        Serial.println("Wrong message size!");
        return false;  
    }
    else {

    int i;
    int val;
    bool val_bool;

    user_input->letter[n_bytes] = '\0';
    String str_msg = String(user_input->letter);

    switch(user_input->numb[0]) {
        //BASIC COMMANDS 
        case duty_msg: //‘u_<i>_<val>’
            //Set directly the duty cycle
            i = get_int_from_str(cut_string(str_msg,2,1));
            val = get_int_from_str(cut_string(str_msg,4,3));
            pwm = val;
            // affect pid 
            usr_flags->pid = false;
            break;
        case get_msg: // 'g xx xx xx'
              switch(user_input->numb[2]) {                    
            //BASIC COMMANDS          
                    case duty_msg: //'g u <i>’ 
                        //Get current duty cycle
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current duty-cycle % : "); Serial.print((curr_data->u*100)/DAC_RANGE);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case ref_msg: //'g r <i>’
                        //Get current illuminance reference of luminaire i
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current ref in lux : "); Serial.print(curr_data->ref); 
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case lux_msg: // g u <i>
                        // Measure the actual illuminance (LUX sensor) of luminaire i
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current measured y in lux : "); Serial.print(curr_data->out);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case volt_msg: //'g v <i>’
                        //Measure the voltage level at node the LDR at node luminaire i
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current measured voltage at ldr : "); ; Serial.print(analogRead(A0)*3.3/DAC_RANGE);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case occupancy_msg: //'g o <i>’
                        //Get the current occupancy state of desk <i>
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current occupancy : "); Serial.print(usr_flags->occupancy);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case a_msg: //'g a <i>’
                        //Get anti-windup state of desk <i>
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current anti-windup : "); Serial.print(usr_flags->anti_windup);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case f_msg: //'g f <i>’
                        //Get feedback control state of desk <i>
                        i = get_int_from_str(cut_string(str_msg,4,1));                    
                        Serial.print("Current feedback state : "); Serial.print(usr_flags->feed_backward);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case dist_msg: //'g d <i>’
                        //Get current external illuminance of desk <i>
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Current disturbance : ");Serial.print(lux_info->min_lux);
                        Serial.print(" at node "); Serial.println(i);    
                        break;
                    case p_msg: //'g p <i>’
                        //Get instantaneous power consumption of desk <i>
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Power consumption : ");Serial.print(perf_data->Pmax*(curr_data->u/DAC_RANGE));
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case time_msg: //'g t <i>’
                        //Get the elapsed time since the last restart
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Time elapsed in ms: ");Serial.print(curr_data->time);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case b_msg: //'g_b_<x>_<i>’
                        //Get the last minute buffer of the variable <x> of the desk <i>. <x> can be 'y' or 'u'.
                        i = get_int_from_str(cut_string(str_msg,6,1));
                        if (user_input->letter[2] == lux_msg) {
                            curr_buff->print_buff(true,i); //true for lux
                        }
                        else if(user_input->letter[2] == duty_msg){
                            curr_buff->print_buff(false,i); //false for pwm
                        }
                        else {
                            Serial.println ("Invalid command g b <x> <i>");
                        }
                        break;
                    //PERFORMANCE COMMANDS
                    case E_msg: //'g E <i>’
                        //Get the average energy consumption at node the desk <i> since the last system restart.
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        Serial.print("Energy consumption : ");Serial.print(perf_data->energy);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case V_msg: //'g V <i>’
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        //Get the average visibility error at node desk <i> since the last system restart.
                        Serial.print("Visibility : ");Serial.print(perf_data->visibility);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    case F_msg: //'g F <i>’
                        i = get_int_from_str(cut_string(str_msg,4,1));
                        //Get the average flicker error on desk <i> since the last system restart.
                        Serial.print("Flicker : ");Serial.print(perf_data->flicker);
                        Serial.print(" at node "); Serial.println(i);
                        break;
                    default:
                        Serial.println ("Invalid command at g 'x' ");
                        break;
                    }
            break;
        case ref_msg: //‘r <i> <val>’
            //Set the illuminance reference of illuminaire i
            i = get_int_from_str(cut_string(str_msg,2,1));
            val = get_int_from_str(cut_string(str_msg,4,2));
            usr_flags->pid = true;
            ref = val;
            Serial.print("New ref : ");Serial.print(val);
            Serial.print(" at node "); Serial.println(i);
            //write to ref
            break;
        case occupancy_msg: //‘o <i> <val>’
            //Set the current occupancy state of desk <i>
            i = get_int_from_str(cut_string(str_msg,2,1));
            val_bool = get_int_from_str(cut_string(str_msg,4,2));
            usr_flags->occupancy = val_bool;
            Serial.print("Occupancy set to "); Serial.print(val_bool);
            Serial.print(" at node "); Serial.println(i);
            break;
        case f_msg: //‘f <i> <val>’
            //Set feedback control on/off on desk <i>        
            i = get_int_from_str(cut_string(str_msg,2,1));
            val_bool = get_int_from_str(cut_string(str_msg,4,2));
            usr_flags->feed_backward = val_bool; 
            Serial.print("Feedback set to "); Serial.print(usr_flags->feed_backward);
            Serial.print(" at node "); Serial.println(i);
            break;
        case a_msg: //‘a <i> <val>’
            //Set anti-windup on/off on at node desk <i>
            i = get_int_from_str(cut_string(str_msg,2,1));
            val_bool = get_int_from_str(cut_string(str_msg,4,2));
            usr_flags->anti_windup = val_bool;
            Serial.print("anti-windup set to"); Serial.print(val_bool);
            Serial.print(" at node "); Serial.println(i);
            break;            
        case data_stream_start_msg: //‘s <x> <i>’
            //Start the stream of the real-time variable <x> of desk <i>. <x> can be 'y' or 'u'.
            i = get_int_from_str(cut_string(str_msg,2,1));
            if (user_input->numb[2] == lux_msg) { //out stream
                usr_flags->rt_stream_out = true;
                Serial.print("Lux stream enabled ");
                Serial.print(" at node "); Serial.println(i);
            }
            else if(user_input->numb[2] == duty_msg){ //u stream
                usr_flags->rt_stream_u = true;
                Serial.print("duty-cycle stream enabled ");
                Serial.print(" at node "); Serial.println(i);   
            }
            else {
                Serial.println ("Invalid command s <x> <i>");
            }
            break;
        case data_stream_stop_msg: //‘S <x> <i>’
            //Stop the stream of the real-time variable <x> of desk <i>. <x> can be 'y' or 'u'
            i = get_int_from_str(cut_string(str_msg,4,1));
            if (user_input->numb[2] == lux_msg) {
                usr_flags->rt_stream_out = false;
                Serial.print("Lux stream disabled ");
                Serial.print(" at node "); Serial.println(i);
            }
            else if(user_input->numb[2] == duty_msg){
                usr_flags->rt_stream_u = false;
                Serial.print("Duty-cycle stream disabled ");
                Serial.print(" at node "); Serial.println(i); 
            }
            else {
                Serial.println ("Invalid command S <x> <i>");
            }
            break;
        default:
            Serial.println ("Invalid command at first letter");
            break;
        }
    }
    return true;
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
        //Serial.println("Caught inner frame!"); 
    }
}

void canbus_comm::send_can(msg_to_can* inner_frame, MCP2515* can) {
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
        inner_frm_to_fifo(inner_frame);
        inner_frame->wrapped.internal_msg[0] = ACK;
    }
}

void canbus_comm::send_msg(uint8_t id, uint8_t header, uint64_t data, msg_to_can* inner_frame) {
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
            can_data_ptr = (uint64_t*)inner_frame->wrapped.can_msg.data;
            // print
            Serial.println("Msg Broadcast!");
            Serial.print("msg id ");Serial.println(inner_frame->wrapped.can_msg.can_id);
            Serial.print("msg dlc ");Serial.println(inner_frame->wrapped.can_msg.can_dlc);
            Serial.print("data ");Serial.println(*can_data_ptr);
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
