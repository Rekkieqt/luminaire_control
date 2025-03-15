#include "ring_buffer.h"
#include <Arduino.h>
#include <cstring>
#include "init.h"

ring_buffer::ring_buffer() {}

void ring_buffer::push(data_reads* data) {
    memcpy(&buffer[tail_i], data, sizeof(data_reads));
    increment(tail_i); //increment after writing
    is_empty = false;
}

data_reads* ring_buffer::get(int offset) { // WHEN CALLED WITH 'TAIL_I' SHOULD RETURN LAST WRITTEN
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
void ring_buffer::print_buff() {
    for (int j = 0 ; j < BUFFER ; j++) {
        data_reads* data_print = get(tail_i +j);
        //Serial Print
        Serial.print("out :"); Serial.print (data_print->out);
        Serial.print("u :"); Serial.println(data_print->u);
        Serial.print("ref :"); Serial.print(data_print->ref);
        Serial.print("time :"); Serial.print (data_print->time);
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