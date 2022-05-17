#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "KeyBot.h"
#include "eigen.h"

namespace Dyn {


  void calculateJacobian(float* q);
  
  extern Eigen::Matrix2f J; // Jacobian matrix

}

#endif
