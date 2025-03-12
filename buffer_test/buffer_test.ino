#include "luxmeter.h"
#include "ring_buffer.h"
#include "pid.h"

enum static_parameters {
  DAC_RES = 12,
  WRITE_FREQ = 60000,
  LED_PIN = 15,
  DAC_RANGE = 4095,
  R1 = 225000
};

int counter = 0;
double G;
luxmeter luxm(R1);
ring_buffer data_buff;
data_reads current_read; 

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin();
  analogReadResolution(DAC_RES);
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  luxm.calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  current_read.out = luxm.get_lux();
  current_read.ref = counter;
  current_read.u = counter*G;
  current_read.time = counter/10;
  data_buff.push(&current_read);

  analogWrite(LED_PIN,counter);
  delay(1);

  counter=counter+1;
  if (counter > DAC_RANGE) {
    counter = 0;
    data_buff.print_buff();
  }

  /*
  Serial.print(0); Serial.print (" ");
  Serial.print(DAC_RANGE); Serial.print (" ");
  Serial.print(read_adc); Serial.print (" ");
  Serial.print(counter); Serial.println();
  */
}
