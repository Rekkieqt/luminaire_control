#include <Arduino.h>
#include "command.h"
#include "performance.h"
#include "ldr.h"
#include "ring_buffer.h"
#include <cstring>
#include <cstddef>
#include <cstdio>
#include "init.h"
#include "comms.h"

char buff[50];

void set_dutycycle(float val){
  int dutycycl {0};
  dutycycl = constrain(val, 0, 100);
  dutycycl = static_cast<int>(dutycycl*(DAC_RANGE-1)/100);
  analogWrite(LED_PIN, dutycycl); // set led PWM
}

void get_command(float v, float L, float u, float& ener, float& flicker, float& vis_err, unsigned long& N, ring_buffer* _data_log, canbus_comm* hermes, msg_to_can* inner_frame){
  int index{0};
  char c{'\0'};
  memset(buff, 0, sizeof(buff));
  while (Serial.available() && index < sizeof(buff)-1){
    c = Serial.read(); 
    if(c == '\n' || c == '\r' || c == '\0'){
      continue;
    }
    else{
      buff[index++] = c;
    }
  }
  char out[20];
  char* token = strtok(buff, " ");
  switch (token[0]){
    case 'g':{
      token = strtok(NULL, " ");
      
      switch(token[0]){
        case 'v':{ // get LDR voltage
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_volt);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);            
          }
          else {
            sprintf(out, "v %d %f", myIdentifier, v);
            Serial.println(out);  
          }
          break;
        }
        case 'u':{ // get control value
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_u);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);            
          }
          else {
            sprintf(out, "u %d %f", myIdentifier, u);
            Serial.println(out);
          }
          break;
        }
        case 'y':{ // get illuminance
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_y);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);            
          }
          else if (atoi(token) > 0 && atoi(token) <= maxId) {
            sprintf(out, "y %d %f", myIdentifier, L);
            Serial.println(out);
          }
          break;
        }
        case 'r':{ // get system reference
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_ref);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);            
          }
          else  {
            sprintf(out, "r %d %f", myIdentifier, PID.get_reference());
            Serial.println(out);
          }
          break;
        }
        case 'o':{ // get occupancy state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_occ);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "o %d %d", myIdentifier, occ_st);
            Serial.println(out);
          }
          break;
        }
        case 'a':{ // get anti-windup state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_aa);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else  {
            sprintf(out, "a %d %d", myIdentifier, PID.get_anti_wu_status());
            Serial.println(out);
          }
          break;          
        }
        case 'f':{ // get feedback state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_fb);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "f %d %d", myIdentifier, PID.get_fb_status());
            Serial.println(out);
          }
          break;          
        }
        case 'd':{ // get external illuminance
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if ((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_dist);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else  {
            sprintf(out, "d %d %f", myIdentifier, PID.get_disturbance(u));
            Serial.println(out);
          }
          break;
        }
        case 'p':{ // get instant power consumption
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if ((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_pwr);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "p %d %f", myIdentifier, inst_power_consumption(uk_1));
            Serial.println(out);
          }
          break;
        }
        case 't':{ // get time elapsed since restart
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if ((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_Rtime);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "t %d %f", myIdentifier, (millis() - last_restart)*1e-3);
            Serial.println(out);
          }
          break;
        }
        case 'b':{ // get last minute buffer
          int i;
          if(sscanf(buff + 4 , "%c %d", &x, &i)!=2){break;}
          
          // argument validation
          if(i < 0) {Serial.println("err"); break;}
          if(x != 'u' && x != 'y') {Serial.println("err"); break;}

          if(i == myIdentifier) print_buff = true;
          break;
        }
        case 'E':{ // get energy
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_energy);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "E %d %f", myIdentifier, ener);
            Serial.println(out);
          }
          break;
        }
        case 'V':{ // get visibility
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),get_vis);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
          }
          else {
            sprintf(out, "V %d %f", myIdentifier, vis_err/N);
            Serial.println(out);
          }
          break;
        }
        case 'F':{ // get flicker
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),(uint8_t)get_flicker);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
            }
          
          else {
            sprintf(out, "F %d %f", myIdentifier, flicker/N);
            Serial.println(out);
          }
          break;
        }
        default:{
          Serial.println("err");
          break;
        }
      }
      break;
    }
    case 'u':{
      int i;
      float val;
      if(sscanf(buff + 2 , "%d %f", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0 || val > 100) {Serial.println("err"); break;}

      if(i == myIdentifier) {
        set_dutycycle(val); 
        Serial.println("ack");
      }
      else if (i <= maxId) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
        hermes->ser_req(inner_frame,(uint8_t)i, (uint8_t)set_u , &val, sizeof(val));
        Serial.print("Request sent to ");Serial.println(i,HEX);
      }
      break;
    }
    case 'r':{
      int i;
      float val;
      if(sscanf(buff + 2 , "%d %f", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0) {Serial.println("err"); break;}
      if(i == myIdentifier) {
        PID.set_reference(val); 
        Serial.println("ack");
      }
      else if (i <= maxId) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
        hermes->ser_req(inner_frame, (uint8_t)i, (uint8_t)set_ref, &val, sizeof(val));
        Serial.print("Request sent to ");Serial.println(i,HEX);
      }
      break;
    }
    case 'o':{
      int i;
      int val;
      bool st;
      float ref;
      if(sscanf(buff + 2 , "%d %d", &i, &val)!=2){Serial.println("err"); break;};

      if(val != 0 && val != 1) {Serial.println("err"); break;}

      if(i < 0) {Serial.println("err"); break;}
      if(val == 0) {st = false; ref = PID.r_l;} else {st = true; ref = PID.r_h;}
      if(i == myIdentifier) {
        occ_st = st; 
        PID.set_reference(ref); 
        Serial.println("ack");
      }
      else if (i <= maxId) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
        hermes->ser_req(inner_frame,(uint8_t)i, set_occ);
        Serial.print("Request sent to ");Serial.println(i,HEX);
      }

      break;
    }
    case 's':{
      token = strtok(NULL, " ");
      
      if(token[0] == 'y'){ // start stream of illuminance measurements
      if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
      if((token) != NULL && atoi(token) != myIdentifier) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
          hermes->ser_req(inner_frame,(uint8_t)atoi(token),start_stream_y);
          Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
        }
      else {
          print_y = true;
        }
      }   
      else if(token[0] == 'u'){ // start stream of control values
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token), start_stream_u);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
        }
        else {
          print_u = true;
        }
      }
      else if(token[0] == 'p'){ // start stream of control values
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {break;}
        SERIAL_PRINTS_0 = true;
      }
      else if(token[0] == 'm'){ // start stream of control values
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {break;}
        SERIAL_PRINTS_1 = true;
      }
      break;
    }
    case 'S':{
      token = strtok(NULL, " ");
      
      if(token[0] == 'y'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
          hermes->ser_req(inner_frame,(uint8_t)atoi(token),stop_stream_y);
          Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
        }
        else  {
          print_y = false;
          Serial.println("ack");
        }
      }   
      else if(token[0] == 'u'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {
            //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
            hermes->ser_req(inner_frame,(uint8_t)atoi(token),stop_stream_u);
            Serial.print("Request sent to ");Serial.println(atoi(token),HEX);    
        }
        else {
          print_u = false;
          Serial.println("ack");
        }
      
      }
      else if(token[0] == 'p'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {break;}
        SERIAL_PRINTS_0 = false;
        Serial.println("ack");
      }
      else if(token[0] == 'm'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != myIdentifier) {break;}
        SERIAL_PRINTS_1 = false;
        Serial.println("ack");
      }
      break;
    }
    case 'a':{
      int i;
      bool val;
      if(sscanf(buff + 2 , "%d %d", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0 || val > 100) {Serial.println("err"); break;}


      if(i == myIdentifier) {
        PID.set_anti_wu_status(val); 
        Serial.println("ack");
      }
      else if (i <= maxId) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
        hermes->ser_req(inner_frame,(uint8_t)i,set_aa,&val, sizeof(val));
        Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
      }
      break;
    }
    case 'f':{
      int i;
      bool val;
      if(sscanf(buff + 2 , "%d %d", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0 || val > 100) {Serial.println("err"); break;}

      if(i == myIdentifier) {
        PID.set_fb_status(val); 
        Serial.println("ack");
      }
      else if (i <= maxId) {
        //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
        hermes->ser_req(inner_frame,(uint8_t)i,set_fb,&val,sizeof(val));
        Serial.print("Request sent to ");Serial.println(atoi(token),HEX);
      }
      break;
    }
    case 'R':{
      //hermes->ser_req(msg_to_can* inner_frame, uint8_t req_id, uint8_t req_cmd);
      hermes->ser_req(inner_frame,(uint8_t)BROADCAST,reset);
      occ_st = UNOCC;
      N = 0;
      ener = 0;
      vis_err = 0;
      flicker = 0;
      _data_log->clear();
      last_restart = time_us_64()/1000;
      adjust_gain();
      PID.set_reference(occ_st ? PID.r_h : PID.r_l);
      PID.set_system_gain_n_dist(G,d);
      Serial.println("ack");
      break;
    }
    case 'z':{ // extra print
      //if(i < 0) {Serial.println("err"); break;}
      dn_special_print = dn_special_print ^ true;
      Serial.println("special ack");
      break;
    }
    default:
      Serial.println("err");
      break;
  }
}