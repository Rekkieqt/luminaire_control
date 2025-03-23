#ifndef LUXMETER_H
#define LUXMETER_H
#include <Arduino.h>
#include "luxmeter.h"
#include "init.h"

class luxmeter 
{
//inline defs
  private:
    int R0;
    int N;
    float gamma; 
    float log_R0;
    volatile float ldr;
    volatile float lux;
    float G;
    float min_lux;
    float max_lux;
  public:
    //constructor
    luxmeter(int _R0,int Nf); 
    //destructor
    ~luxmeter();
    //function
    float get_ldr();
    float get_lux();
    float get_dist();
    void get_lux_data(static_lux_data* _lux_data);
    
    void get_offset();
    void tf_sweep();
    void calibrate();
    
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
