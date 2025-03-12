#include "luxmeter.h"

enum static_parameters {
  DAC_RES = 12,
  WRITE_FREQ = 60000,
  LED_PIN = 15,
  DAC_RANGE = 4095,
  R1 = 225000
};

int counter = 0;
luxmeter luxm(R1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(DAC_RES);
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  double G = luxm.get_offset();
  luxm.tf_sweep();
  luxm.calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  int read_adc;
  analogWrite(LED_PIN,0);
  delay(1);
  read_adc=analogRead(A0); //analog pin ADC0
  counter=counter+1;
  if (counter > DAC_RANGE) counter = 0;
  /*
  Serial.print(0); Serial.print (" ");
  Serial.print(DAC_RANGE); Serial.print (" ");
  Serial.print(read_adc); Serial.print (" ");
  Serial.print(counter); Serial.println();
  */
}
