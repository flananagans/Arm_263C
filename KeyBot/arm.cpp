#include "arm.h"

/*
 * Interface for the SerialArm object
 *  Get the current state of the joints
 *  Set the desired voltage of each joint
 */
namespace Arm {
  
  SerialArm arm(&PORT_ARM, 36);
  float V_MAX = 12.0;

  // Current position
  float curr_q[2] = {0};
  // Current velocity
  float curr_dq[2] = {0};
  // Current time
  float curr_t = 0.0;

  void start() {
    
    for(uint8_t j = 0; j < NUM_JOINTS; j++) {
      arm.addJoint(j + 1);
    }
  
    arm.start();
    // Configure with delay of 0us (2*var), PWM mode, max voltage 
    arm.configure(0, 16, V_MAX);

    disable();
  }

  void enable() {
    arm.enable();
  }

  void disable() {
    arm.disable();
  }

  /*
   * Update position and velocity of the arm
   *  Access values through two extern arrays
   */
  bool updateState() {

    curr_t = (micros() - KeyBot::t0)/(1e6); // time in ms
    arm.getAllPos(&curr_q[0]);
    arm.getAllVel(&curr_dq[0]);

    if(isnan(curr_q[0]) || isnan(curr_dq[0])) {
      return false;
    }

    Serial.print("q: ");
    Serial.print(curr_q[0]);
    Serial.print(",");
    Serial.println(curr_q[1]);
    
    if(SDCARD) {
      // Save to SD
    }
    return true;
  }

  /*
   * Set the voltage of the motors
   *  v_d should be a pointer to a 1 x arm.numJoints float array 
   */
  void setV(float* v_d) {
    arm.setV(v_d);
    
    Serial.print("v: ");
    Serial.print(v_d[0]);
    Serial.print(",");
    Serial.println(v_d[1]);
    
    if(SDCARD) {
      // Save to SD
    }
  }
}
