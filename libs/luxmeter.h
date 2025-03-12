#ifndef LUXMETER_H
#define LUXMETER_H

class luxmeter 
{
//inline defs
  private:
    int resistor;
    int R1;
    float gamma; 
    float vcc;
    double ldr;
    double lux_offset;
    double lux;
    double G;
    int N;
  public:
    //constructor
    luxmeter(int R1);
    //int resistor, float gamma, float vcc, double ldr, double lux_offset, double lux, double G  
    //destructor
    ~luxmeter();
    //function
    double get_ldr();
    double get_lux();

    void get_offset();
    void tf_sweep();
    double calibrate();
    
};

class performance 
{
//inline defs
  private:
    int Pmax;
    int R1;
    float gamma; 
    float vcc;
    double ldr;
    double lux_offset;
    double lux;
    double G;
    int N;
  public:
    //constructor
    explicit performance(int _Pmax);
    //destructor
    ~performance();
    //function
    double get_energy();
    double get_visibility();
    double get_flicker();
    
};

//function calls 
/*

*/
#endif //LUXMETER_H
