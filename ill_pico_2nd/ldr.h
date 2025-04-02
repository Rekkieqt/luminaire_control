#ifndef LDR_H
#define LDR_H

extern float G;
extern float d;

float get_ldr_voltage(int pinNumber);
float luxmeter(float v);
float* get_ldr_params();
void adjust_gain(void);

#endif
