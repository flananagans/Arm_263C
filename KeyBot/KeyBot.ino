#include <SerialArm.h>

SerialArm arm(&Serial1, 36);

long t0;
float V_MAX = 12.0;

void setup() {
  arm.addJoint(0x01);
  arm.addJoint(0x02);

  arm.start();
  // Configure with delay of 0us (2*var), PWM mode, max voltage 
  arm.configure(0, 16, V_MAX);
  Serial.begin(115200);
  while(!Serial) {};

  //Serial.print("Arm configured with ");
  //Serial.print(arm.getNumJoints());
  //Serial.println(" joints");

  arm.enable();

  t0 = millis();

  float v[2] = {0.0, 0.0};
  arm.setV(&v[0]);
}

void loop() {
  float phs = (2*PI)*fmod( (millis() - t0)/1000.0, 1.0);
  float V_goal[2] = {V_MAX/2*cos(phs), V_MAX/2*sin(phs)};
  long tmr = micros();
  //arm.setV(&V_goal[0]);
  //delayMicroseconds(500);
  //float pos1 = arm.getPos(0);
  //float pos2 = arm.getPos(1);
  float pos_arr[2] = {0};
  arm.getAllPos(&pos_arr[0]);
  //float vel = arm.getVel(1);
  char buffer[64];
  int n = sprintf(&buffer[0], "%0.3f,%0.3f,%0.3f,%0.3f", 
        0, pos_arr[0], 0, pos_arr[1]);
  Serial.println(buffer);
  tmr = micros() - tmr;
  Serial.println(tmr);
  //delayMicroseconds(2000-tmr);
  delay(5000);
}
