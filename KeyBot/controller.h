#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "KeyBot.h"
#include "arm.h"
#include "dynamics.h"
#include "kinematics.h"

namespace Controller {

  void start(void);
  void setQGoal(float* q);

  // Controllers
  void simplePD_JS();

  extern bool goal_reached;

}

#endif
