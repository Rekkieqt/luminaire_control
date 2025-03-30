#ifndef LDR_H
#define LDR_H

extern float G;
extern float d;

float get_ldr_voltage(int pinNumber);
float luxmeter(float v);
void adjust_gain(void);

#endif
