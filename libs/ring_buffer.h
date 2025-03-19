#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#include "init.h"

class ring_buffer {
private:
    //data_reads* data_print = NULL;
    data_reads buffer[BUFFER];
    int head_i = 0;
    int tail_i = 0;
    bool is_empty = true;
public:
    ring_buffer();
    
    void push(data_reads* data);
    
    int pop();
    
    data_reads* get(int offset);
    
    void clear();
    
    void increment(int& i) const;
    
    void decrement(int& i) const;

    void print_buff(bool raw = false);
};


#endif /* RING_BUFFER_H */