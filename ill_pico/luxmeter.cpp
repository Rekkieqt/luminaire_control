#include <Arduino.h>
#include "luxmeter.h"
#include "init.h"

luxmeter::luxmeter(
  int _R0, int Nf)
    : gamma{0.8}, ldr{0}, log_R0{0.0},
      lux{0.0}, G{0.0}, N{Nfilter}, R0{_R0}, min_lux{0}, max_lux{0} {
    get_offset();  
}

float luxmeter::get_ldr() {
  int div_voltage;
  float ldr_est[N];
  for (int i = 0; i < N ; i++) {
    div_voltage = analogRead(A0);
    ldr_est[i] = (R1*(DAC_RANGE - div_voltage + 1))/float(div_voltage);
  }
  isort(ldr_est,sizeof(ldr_est)/sizeof(float));
  ldr = ldr_est[int(N/2)];
  return ldr;
}

void luxmeter::isort(float arr[], int size) {
    for (int i = 1; i < size; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

void luxmeter::get_offset() {
  log_R0 = log10(R0) + gamma;
}

float luxmeter::get_lux() {
  ldr = get_ldr();
  lux = pow(10,( (log_R0) - log10(ldr) )/gamma) ;
  return lux;
}

void luxmeter::tf_sweep() {
  for (int i = 0; i < DAC_RANGE; i = i + 2) {
    //write to voltage pin
    analogWrite(LED_PIN,i);
    delay(50);
    ldr = get_ldr();
    lux = get_lux(); 
    
    //serial print pwm
    Serial.print(" u "); Serial.println(i);
    //serial voltage read
    Serial.print(" volt read "); Serial.println(analogRead(A0));
    //serial print lux
    Serial.print(" out "); Serial.println(lux);
    //serial print ldr
    Serial.print(" Rldr "); Serial.println(ldr);  
  }
}

void luxmeter::calibrate() {
  //write 0 get lux

  unsigned long int curr_time = millis();
  while (millis() - curr_time < ONE_SEC_MS) {
    analogWrite(LED_PIN,int(0.2*DAC_RANGE));
  }
  min_lux = get_lux();
  curr_time = millis();
  while (millis() - curr_time < ONE_SEC_MS) { 
    analogWrite(LED_PIN,DAC_RANGE);
    max_lux = get_lux();
  }
  G = (max_lux-min_lux)/float(DAC_RANGE);  
}

void luxmeter::get_lux_data(static_lux_data* _lux_data) {
  _lux_data->min_lux = min_lux;
  _lux_data->max_lux = max_lux;
  _lux_data->G = G;
}

performance::performance()
: Pmax{0.0}, miu{0,0,0}, time{0,0}, ref{0,0}, beta{1.0f/DAC_RANGE},
      energy{0}, visibility{0}, flicker{0} { }

float performance::get_energy(data_reads dk) {
  time[1] = dk.time;
  energy = Pmax*miu[1]*beta*(time[1] - time[0]) + energy;
  time[0] = time[1];
  return energy;
}

float performance::get_visibility(data_reads dk) {
  //instead of waiting for delay() when ref k != ref k-1, initiate timer
  ref[1] = dk.ref;
  float vk {max(0,ref[1] - dk.out)}; 
  visibility = (vk + visibility)/2;
  ref[1] = ref[0];
  return visibility;
}

float performance::get_flicker(data_reads dk) {
  float fk{0};
  miu[2] = dk.u;
  float a {miu[2] - miu[1]};
  float b {miu[1] - miu[0]};
  if (a*b < 0){
    fk = abs(a) + abs(b);
  }
  else {
    fk = 0;
  }
  flicker = (flicker + fk)/2;
  miu[0] = miu[1];
  miu[1] = miu[2];
  return flicker;
}

perf_meas performance::get_perf(data_reads dk) {
  perf_meas data;
  data.energy = get_energy(dk);
  data.visibility = get_visibility(dk);
  data.flicker = get_flicker(dk);
  data.Pmax = Pmax;
  return data;
}

void performance::get_pmax() {
  unsigned long int curr_time = millis();
  int vled{0};
  float conversor = 3.3f/DAC_RANGE;
  while (millis() - curr_time < ONE_SEC_MS) { 
    analogWrite(LED_PIN,DAC_RANGE);
    vled = analogRead(A1); //voltage at led
  }
  Pmax = pow(((DAC_RANGE - vled)*conversor),2)/Rled;
  curr_time = millis();
    while (millis() - curr_time < ONE_SEC_MS) { 
    analogWrite(LED_PIN,0);
  }
}

float performance::print_pmax() {
  return Pmax;
}