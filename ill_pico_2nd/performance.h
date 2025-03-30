#ifndef PERF_H
#define PERF_H

#include "init.h"
//#include "CircularBuffer/CircularBuffer.hpp"

extern float ener;
extern float flicker;
extern float vis_err;
extern unsigned long N;

extern float uk_1;

//extern CircularBuffer<sample,60*100> last_min_buffer;

float inst_power_consumption(float dk_1);
void update_power_consumption(float power);

void update_vis_err(float lk, float Lk);

void update_flicker(float uk, float uk_1, float uk_2);

#endif