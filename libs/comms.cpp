#include "comms.h"
#include <Arduino.h>
#define LED_PIN 15

communicator::communicator() {
  i = 0;
  val = 0.0;
  time = 0.00;
  command = NULL;
}

communicator::~communicator() {
  
}

double communicator::get_() {

}

void communicator::get_() {
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
void communicator::decoder(int* msg, int bytes) {
  for (int i = 0; i < bytes; i++) {
    commmand[i] = *(msg + i);
    }


  switch(commmand[0]); {
  //BASIC COMMANDS
        case u_msg:
            func(commmand[1], commmand[2]);
            break;
        case g_msg:
              switch(commmand[1]); {
              //PERFORMANCE COMMANDS
                    case E_msg:
                        func(commmand[1]);
                        break;
                    case V_msg:
                        func(commmand[1]);
                        break;
                    case F_msg:
                        func(commmand[1]);
                        break;                    
              //BASIC COMMANDS          
                    case u_msg:
                        func(commmand[1]);
                        break;
                    case r_msg:
                        func(commmand[1]);
                        break;
                    case y_msg:
                        func(commmand[1]);
                        break;
                    case v_msg:
                        func(commmand[1]);
                        break;
                    case o_msg:
                        func(commmand[1]);
                        break;
                    case a_msg:
                        func(commmand[1]);
                        break;
                    case f_msg:
                        func(commmand[1]);
                        break;
                    case d_msg:
                        func(commmand[1]);
                        break;
                    case p_msg:
                        func(commmand[1]);
                        break;
                    case t_msg:
                        func(commmand[1]);
                        break;
                    case b_msg:
                        func(commmand[1], commmand[2]);
                        break;

                    default:
                    //error occurred or sum
                        break;
    }
            break;
        case r_msg:
            func(commmand[1], commmand[2]);
            break;
        case o_msg:
            func(commmand[1], commmand[2]);
            break;
        case f_msg:        
            func(commmand[1], commmand[2]);
            break;
        case a_msg:
            func(commmand[1], commmand[2]);
            break;            
        case s_msg:
            func(commmand[1], commmand[2]);
            break;
        case S_msg:
            func(commmand[1], commmand[2]);
            break;
        default:
            //error occurred or sum
            break;
    }
}