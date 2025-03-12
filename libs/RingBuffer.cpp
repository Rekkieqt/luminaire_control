//
//  RingBuffer.cpp
//
//  Created by Jacob Schwartz on 10/23/17.
//

#include "RingBuffer.hpp"
#include <cassert>

RingBuffer::RingBuffer() {}

void RingBuffer::push(int val) {
    buffer[tail_i] = val;
    increment(tail_i);
    
    is_empty = false;
}

int RingBuffer::pop() {
    assert(tail_i - head_i > 0);
    
    int val = buffer[tail_i];
    decrement(tail_i);
    
    if (tail_i == head_i)
        is_empty = true;
    
    return val;
}

int RingBuffer::get(int offset) {
    assert(offset < BUF_SIZE);
    assert(head_i != tail_i);
    
    if (head_i + offset > BUF_SIZE)
        return buffer[head_i + offset - BUF_SIZE];
    
    return buffer[head_i + offset];
}

void RingBuffer::clear() {
    head_i = 0;
    tail_i = 0;
}

void RingBuffer::increment(int& i) const {
    i = (i + 1 < BUF_SIZE) ? (i + 1) : (0);
}

void RingBuffer::decrement(int& i) const {
    i = (i - 1 >= 0) ? (i - 1) : (BUF_SIZE - 1);
}

void RingBuffer::print_buff() {
    int temp;
    for ( j = 0 ; j < BUF_SIZE ; j++) {
        //Serial Print
        temp = get(tail_i + j);
    }
}