#ifndef LUXMETER_H
#define LUXMETER_H

class luxmeter 
{
//inline defs
  private:
    int resistor;
    float gamma; 
    float vcc;
    double ldr;
    double lux_offset;
    double lux;
    double G;
  public:
    //constructor
    luxmeter();
    //int resistor, float gamma, float vcc, double ldr, double lux_offset, double lux, double G  
    //destructor
    ~luxmeter();
    //function
    double get_ldr();
    double get_lux();

    void get_offset(int R1);
    void tf_sweep();
    void calibrate();
    
};

//function calls 
/*
  double get_offset(int a, int b);
  void tf_sweep();
  void calibrate();
    
*/
#endif //LUXMETER_H
