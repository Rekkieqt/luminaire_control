//
//  RingBuffer.hpp
//
//  Created by Jacob Schwartz on 10/23/17.
//

#ifndef RingBuffer_hpp
#define RingBuffer_hpp

class RingBuffer {
private:
    const static int BUF_SIZE = 10;
    int buffer[BUF_SIZE];
    int head_i = 0;
    int tail_i = 0;
    bool is_empty = true;
public:
    RingBuffer();
    
    void push(int val);
    
    int pop();
    
    int get(int offset);
    
    void clear();
    
    void increment(int& i) const;
    
    void decrement(int& i) const;

    void print_buff();
};

#endif /* RingBuffer_hpp */