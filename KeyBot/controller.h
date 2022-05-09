#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "KeyBot.h"
#include "arm.h"
#include "dynamics.h"
#include "kinematics.h"
#include "sdcard.h"

namespace Controller {

  void start(void);
  void setQGoal(float* q);

  // Controllers
  void simplePD_JS();

  extern bool goal_reached;
  extern float t;
  extern float q_arr[2];
  extern float v_arr[2];
}

#endif
