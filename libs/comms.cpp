#include "comms.h"
#include <Arduino.h>
#include "luxmeter.h"
#include "pid.h"
#include "init.h"

communicator::communicator() {

}

communicator::~communicator() {
}

void communicator::func_test(int msg1, float msg2) {
    Serial.println("Working msg detectin"); 
}

void communicator::decode(void* user_input, int n_bytes, luxmeter &meas, data_reads &curr_data, ring_buffer &curr_buff) {
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

/*
void communicator::decoder(int* msg, int bytes) {

  std::vector<std::vector<int>> command;
  
  // Calculate the size of each slice
  
  // Slice the buffer into n parts and store them in vectors
  for (int i = 0; i < bytes; ++i) {
      std::vector<int> slice(buffer + i * sliceSize, buffer + (i + 1) * sliceSize);
      command.push_back(slice);
  }
*/