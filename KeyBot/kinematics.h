#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "KeyBot.h"

namespace Kine {

  bool fkine(float* q);
  bool ikine(float* p);

  extern float last_q[NUM_JOINTS]; // last calculated joint angle
  extern float last_p[2]; // last calculated end position [x,y]
}

#endif
