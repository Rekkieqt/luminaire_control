#include <Arduino.h>
#include "command.h"
#include "performance.h"
#include "ldr.h"
#include "ring_buffer.h"
#include <cstring>
#include <cstddef>
#include <cstdio>
#include "init.h"

char buff[50];

void set_dutycycle(float val){
  int dutycycl {0};
  dutycycl = constrain(val, 0, 100);
  dutycycl = static_cast<int>(dutycycl*(DAC_RANGE-1)/100);
  analogWrite(LED_PIN, dutycycl); // set led PWM
}

void get_command(float v, float L, float u, float ener, float flicker, float vis_err, unsigned long N, ring_buffer* _data_log){
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
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "v %d %f", CIRC_NUM, v);
          Serial.println(out);
          break;
        }
        case 'u':{ // get control value
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "u %d %f", CIRC_NUM, u);
          Serial.println(out);
          break;
        }
        case 'y':{ // get illuminance
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "y %d %f", CIRC_NUM, L);
          Serial.println(out);
          break;
        }
        case 'r':{ // get system reference
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "r %d %f", CIRC_NUM, PID.get_reference());
          Serial.println(out);
          break;
        }
        case 'o':{ // get occupancy state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "o %d %d", CIRC_NUM, occ_st);
          Serial.println(out);
          break;
        }
        case 'a':{ // get anti-windup state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "a %d %d", CIRC_NUM, PID.get_anti_wu_status());
          Serial.println(out);
          break;          
        }
        case 'f':{ // get feedback state
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "f %d %d", CIRC_NUM, PID.get_fb_status());
          Serial.println(out);
          break;          
        }
        case 'd':{ // get external illuminance
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "d %d %f", CIRC_NUM, PID.get_disturbance(u));
          Serial.println(out);
          break;
        }
        case 'p':{ // get instant power consumption
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "p %d %f", CIRC_NUM, inst_power_consumption(uk_1));
          Serial.println(out);
          break;
        }
        case 't':{ // get time elapsed since restart
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "t %d %f", CIRC_NUM, (millis() - last_restart)*1e-3);
          Serial.println(out);
          break;
        }
        case 'b':{ // get last minute buffer
          int i;
          if(sscanf(buff + 4 , "%c %d", &x, &i)!=2){break;}
          
          // argument validation
          if(i < 0) {Serial.println("err"); break;}
          if(x != 'u' && x != 'y') {Serial.println("err"); break;}

          if(i == CIRC_NUM) print_buff = true;
          break;
        }
        case 'E':{ // get time elapsed since restart
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "E %d %f", CIRC_NUM, ener);
          Serial.println(out);
          break;
        }
        case 'V':{ // get time elapsed since restart
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "V %d %f", CIRC_NUM, vis_err/N);
          Serial.println(out);
          break;
        }
        case 'F':{ // get time elapsed since restart
          if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
          if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
          sprintf(out, "F %d %f", CIRC_NUM, flicker/N);
          Serial.println(out);
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

      if(i == CIRC_NUM) set_dutycycle(val);
      Serial.println("ack");
      break;
    }
    case 'r':{
      int i;
      float val;
      if(sscanf(buff + 2 , "%d %f", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0) {Serial.println("err"); break;}

      if(i == CIRC_NUM) PID.set_reference(val);
      Serial.println("ack");
      break;
    }
    case 'o':{
      int i;
      int val;
      bool st;
      float ref;
      if(sscanf(buff + 2 , "%d %d", &i, &val)!=2){Serial.println("err"); break;};

      if(i < 0) {Serial.println("err"); break;}
      if(val != 0 && val != 1) {Serial.println("err"); break;}
      if(val == 0) {st = false; ref = PID.r_l;} else {st = true; ref = PID.r_h;}

      if(i == CIRC_NUM) {occ_st = st; PID.set_reference(ref);}
      Serial.println("ack");
      break;
    }
    case 's':{
      token = strtok(NULL, " ");
      
      if(token[0] == 'y'){ // start stream of illuminance measurements
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
        print_y = true;
      }   
      else if(token[0] == 'u'){ // start stream of control values
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
        print_u = true;
      }
      break;
    }
    case 'S':{
      token = strtok(NULL, " ");
      
      if(token[0] == 'y'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
        print_y = false;
        Serial.println("ack");
      }   
      else if(token[0] == 'u'){ // get LDR voltage
        if((token = strtok(NULL, " ")) == NULL) {Serial.println("err"); break;}
        if((token) != NULL && atoi(token) != CIRC_NUM) {break;}
        print_u = false;
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

      if(i == CIRC_NUM) PID.set_anti_wu_status(val);
      Serial.println("ack");
      break;
    }
    case 'f':{
      int i;
      bool val;
      if(sscanf(buff + 2 , "%d %d", &i, &val)!=2){Serial.println("err"); break;};

      // argument validation
      if(i < 0) {Serial.println("err"); break;}
      if(val < 0 || val > 100) {Serial.println("err"); break;}

      if(i == CIRC_NUM) PID.set_fb_status(val);
      Serial.println("ack");
      break;
    }
    case 'R':{
      occ_st = UNOCC;
      N = 0;
      ener = 0;
      vis_err = 0;
      flicker = 0;
      _data_log->clear();
      last_restart = time_us_64()/1000;
      adjust_gain();
      PID.set_reference(occ_st ? PID.r_h : PID.r_l);
      PID.set_system_gain(G);
      Serial.println("ack");
      break;
    }
    default:
      Serial.println("err");
      break;
  }
}