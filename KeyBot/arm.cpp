#include "KeyBot.h"
#include "arm.h"

namespace Arm{
  
  SerialArm arm(&PORT_ARM, 36);
  float V_MAX = 12.0;

  // Current position
  float curr_pos[2] = {0};
  // Current velocity
  float curr_vel[2] = {0};

  void start() {
    arm.addJoint(0x01);
    arm.addJoint(0x02);
  
    arm.start();
    // Configure with delay of 0us (2*var), PWM mode, max voltage 
    arm.configure(0, 16, V_MAX);
  }

  void enable() {
    arm.enable();
  }

  void disable() {
    arm.disable();
  }

  void getCurrentState() {
    arm.getAllPos(&curr_pos[0]);
    arm.getAllVel(&curr_vel[0]);
  }
}
