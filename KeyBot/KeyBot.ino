#include "KeyBot.h"

long t0;

void setup() {
  
  Serial.begin(115200);
  while(!Serial) {};
  
  t0 = micros();
}

void chSetup() {}

void loop() {
  long tmr = micros();

  tmr = micros() - tmr;
  delayMicroseconds(2000-tmr);
}
