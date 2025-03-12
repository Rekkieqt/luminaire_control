//
//  buffer.hpp
//
//  Created by Jacob Schwartz on 10/23/17.
//

#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#define BUFFER 100;

struct data_reads {
    double time;
    double out;
    int ref;
    double u;
};

class ring_buffer {
private:
    const static int BUF_SIZE = BUFFER;
    //data_reads* data_print = NULL;
    data_reads buffer[BUF_SIZE];
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

    void print_buff();
};


#endif /* RING_BUFFER_H */