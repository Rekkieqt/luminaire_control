#include "luxmeter.h"
#include "pid.h"
#include "ring_buffer.h"

enum static_parameters {
  DAC_RES = 12,
  WRITE_FREQ = 60000,
  LED_PIN = 15,
  DAC_RANGE = 4095,
  R1 = 225000,
  Fs = 10000,
  Freq = 100
};


int x = 0;
double G{0};
luxmeter luxm(R1);
pid wallee(1/Fs);

void setup() {
  // put your setup code here, to run once:
  int R1 = 225000;
  Serial.begin(115200);
  analogReadResolution(DAC_RES);
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  G = luxm.calibrate();
}

void loop() {

  float ref = 20*sin(Freq*(x*0.05)) + 30;
  double out = luxm.get_lux();
  double miu = wallee.feed_backward(ref,out,G);
  analogWrite(LED_PIN,miu);
  wallee.housekeep(ref);
  delay(1);
  Serial.println(ref); 
  Serial.println(miu); 
  Serial.println(out);

}
