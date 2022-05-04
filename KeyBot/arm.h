#ifndef ARM_H
#define ARM_H

#include <SerialArm.h>

namespace Arm{

  void start(void);
  void enable(void);
  void disable(void);
  void getCurrentState(void);

  extern float curr_pos[2];
  extern float curr_vel[2];

}

#endif
