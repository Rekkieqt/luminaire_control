#ifndef LUXMETER_H
#define LUXMETER_H
#include "ring_buffer.h"
#include "init.h"

class luxmeter 
{
//inline defs
  private:
    int resistor;
    int R1;
    float gamma; 
    float vcc;
    float ldr;
    float lux_offset;
    float lux;
    float G;
    float min_lux;
    int N;
  public:
    //constructor
    luxmeter(int R1,int N);
    //int resistor, float gamma, float vcc, float ldr, float lux_offset, float lux, float G  
    //destructor
    ~luxmeter();
    //function
    float get_ldr();
    float get_lux();
    float get_dist();
    
    void get_offset();
    void tf_sweep();
    float calibrate();
    
};

class performance 
{
//inline defs
  private:
    int Pmax;
    float miu[3];
    float time[2]; 
    float ref[2];
    float beta;
    float energy;
    float visibility;
    float flicker;
  public:
    //constructor
    explicit performance(int _Pmax);
    //destructor
    ~performance();
    //function
    float get_energy(data_reads dk);
    float get_visibility(data_reads dk);
    float get_flicker(data_reads dk);
    
};

//function calls 
/*

*/
#endif //LUXMETER_H
