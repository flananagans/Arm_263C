#include <SerialArm.h>

SerialArm arm(&Serial1, 36);

long t0;
float V_MAX = 12.0;

void setup() {
  arm.addJoint(0x01);
  arm.addJoint(0x02);

  arm.start();
  // Configure with delay of 100us (2*var), PWM mode, max voltage 
  arm.configure(50, 16, V_MAX);
  Serial.begin(115200);
  while(!Serial) {};

  Serial.print("Arm configured with ");
  Serial.print(arm.getNumJoints());
  Serial.println(" joints");

  arm.enable();

  t0 = millis();
}

void loop() {
  float phs = (2*PI)*fmod( (millis() - t0)/10000.0, 1.0);
  float V_goal[2] = {V_MAX*cos(phs), V_MAX*sin(phs)};
  V_goal[0] = 0.0;
  V_goal[1] = 0.0;
  long tmr = micros();
  arm.setV(&V_goal[0]);
  tmr = micros() - tmr;
  Serial.println(tmr);
  delay(1);
}
