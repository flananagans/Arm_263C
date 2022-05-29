#ifndef ARM_H
#define ARM_H

#include "KeyBot.h"

#include <SerialArm.h>

namespace Arm {

  void start(void);
  void enable(void);
  void disable(void);
  bool updateState(void);
  void setV(float* v_d);
  void pressKey(void);
  void releaseKey(void);

  extern float curr_q[NUM_JOINTS]; // current joint position
  extern float curr_dq[NUM_JOINTS]; // current joint velocity
  extern float curr_t; // current time
  extern bool enabled;
}

#endif
