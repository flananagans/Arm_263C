#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "KeyBot.h"
#include "arm.h"
#include "dynamics.h"
#include "kinematics.h"
#include "trajectory.h"
#include "sdcard.h"

namespace Controller {

  void start(void);
  void setQGoal(float* q, float* dq, float* ddq);
  void setPGoal(float* p);

  // Controllers
  void simplePID_JS();
  void simplePID_TS();
  void ID_TS();

  extern volatile bool goal_reached;
  extern float t;
  extern float q_arr[2];
  extern float v_arr[2];
}

#endif
