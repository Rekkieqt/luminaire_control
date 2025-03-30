#include <Arduino.h>
#include "performance.h"


float P_max = (CIRC_NUM == 1) ? 0.0173 : // I_LED = 306mV/47Ohm, V_LED = 2.66; P = VI = 0.0173W
              (CIRC_NUM == 2) ? 0.0177 : // I_LED = 312mV/47Ohm, V_LED = 2.67; P = VI = 0.0177W
              0.0;
float inst_power_consumption(float dk_1){
  return P_max*dk_1;
};

void update_power_consumption(float power){
  ener += power;
}

void update_vis_err(float lk, float Lk){
  vis_err += max(0, Lk - lk);
}

void update_flicker(float uk, float uk_1, float uk_2){
  if((uk-uk_1)*(uk_1-uk_2) < 0)
    flicker += abs(uk-uk_1)+abs(uk_1-uk_2);
}
