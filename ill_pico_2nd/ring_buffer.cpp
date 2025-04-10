#include <Arduino.h>
#include <cstring>
#include "ring_buffer.h"


ring_buffer::ring_buffer() {}

void ring_buffer::push(data_reads* data) {
    memcpy(&buffer[tail_i], data, sizeof(data_reads));
    increment(tail_i); //increment after writing
    is_empty = false;
}

data_reads* ring_buffer::get(int offset) { // WHEN CALLED WITH 'TAIL_I' WILL RETURN LAST WRITTEN
    if (head_i + offset >= BUFFER)
        return &buffer[head_i + offset - BUFFER]; // pointer to struct
    return &buffer[head_i + offset]; // pointer to struct
}

void ring_buffer::increment(int& i) const {
    i = (i + 1 < BUFFER) ? (i + 1) : (0);
}
void ring_buffer::decrement(int& i) const {
    i = (i - 1 >= 0) ? (i - 1) : (BUFFER - 1);
}

void ring_buffer::clear() {
    memset(buffer,0 , sizeof(buffer));
}

void ring_buffer::print_buff(char _x, int node) {
    if (_x == 'y') {
        Serial.print("Measured lux at node : "); 
        Serial.println(node);
        for (int j = 0 ; j < BUFFER ; j++) {
            data_reads* data_print = get(tail_i +j);
            //Serial Print
            Serial.print(data_print->out); Serial.print(" "); 
    }
}
    else if (_x == 'u') {
        Serial.print("Measured pwm at node : "); 
        Serial.println(node);
        for (int j = 0 ; j < BUFFER ; j++) {
            data_reads* data_print = get(tail_i +j);
            //Serial Print
            Serial.print(data_print->u);Serial.print(" ");
        }
    }
    else {
        Serial.print(" Error at argument validation ");Serial.println(_x);
    }
}

/*
int ring_buffer::pop() { // SHOUDLNT NEED THIS 
    //assert(tail_i - head_i > 0);
    
    int val = buffer[tail_i];
    decrement(tail_i);
    
    if ((BUF_SIZE - 1) == head_i)
        is_empty = true;
    
    return val;
}
void ring_buffer::pop(data_reads* data) {
    if (is_empty) {
        return; // Nothing to pop
    }

    memcpy(data, &buffer[head_i], sizeof(data_reads));  // Copy data to output
    increment(head_i); // Move head forward

    if (head_i == tail_i) {
        is_empty = true; // Buffer is now empty
    }
}
void ring_buffer::clear() {
    head_i = 0;
    tail_i = BUF_SIZE - 1;
}

void ring_buffer::push(int val) {
    increment(tail_i); //INCREMENT THEN WRITE, 
    buffer[tail_i] = val; // TAIL_I points to the last element written
    
    is_empty = false;
}
*/