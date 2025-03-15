#include "luxmeter.h"
#include "pid.h"
#include "init.h"
#include "ring_buffer.h"

int x = 0;
float  G{0}, dist{0};
struct repeating_timer timer;
int temp_buff[2];

data_reads current_data;
luxmeter luxm(R1, Nfilter);
pid wallee(0.000001);

bool control_seq( struct repeating_timer *t ){
  // control sequence
    Serial.println("Working interrrupt"); 
    return true;
}

void setup() {
  // put your setup code here, to run once:
  add_repeating_timer_ms( -10, control_seq, NULL, &timer); //100 Hz
  Serial.begin();
  analogReadResolution(DAC_RES);
  analogWriteFreq(WRITE_FREQ);
  analogWriteRange(DAC_RANGE);
  G = luxm.calibrate();
  dist = luxm.get_dist();
}

void loop() {
  
  float ref = 20*sin(Freq*(x*0.005)) + 30;
  float out = luxm.get_lux();
  float miu = wallee.feed_backward(ref, out, G, dist);
  analogWrite(LED_PIN,miu);
  wallee.housekeep(ref);
  delay(1);
  Serial.print("ref "); Serial.println(ref); 
  Serial.print("u "); Serial.println(miu); 
  Serial.print("y "); Serial.println(out);
  x = x + 1;
  if(Serial.available()){
    //Serial.readBytes(&temp_buff, sizeof(x));
    Serial.println("Waiting for a min ...");
    delay(60000);
  }
}
