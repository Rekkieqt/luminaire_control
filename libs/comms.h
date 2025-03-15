#ifndef communicator_H
#define communicator_H
#include "init.h"
#include "luxmeter.h"
#include "pid.h"

class communicator 
{
//inline defs
  private:
    msg_read decoder;
  public:
    //constructor
    communicator();
    //destructor
    ~communicator();

    //function
    void decode(void* user_input,int n_bytes, luxmeter &meas, data_reads &curr_data, ring_buffer &curr_buff);
    void func_test(int msg1 = 0, float msg2 = 0);   
};

//function calls 
/*
  float get_offset(int a, int b);
  void tf_sweep();
  void calibrate();
    
*/
#endif //communicator_H
