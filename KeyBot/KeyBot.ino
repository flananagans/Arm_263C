#include <SerialArm.h>

SerialArm arm(&Serial1, 36);

void setup() {
  arm.addJoint(0x01);
  arm.addJoint(0x02);

  arm.start();
  Serial.begin(9600);
  while(!Serial) {};

  Serial.print("Arm configured with ");
  Serial.print(arm.getNumJoints());
  Serial.println(" joints");
}

void loop() {
  while(arm.serPort->available()) {
    arm.serPort->clear();
    //Serial.print(arm.serPort->read(), HEX);
    //Serial.print(',');
  }
  //Serial.println();

  arm.toggleLED(0);
  arm.toggleLED(1);
  delay(1000);
}
