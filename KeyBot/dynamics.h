#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "KeyBot.h"
#include "eigen.h"

namespace Dyn {

  void calculateJacobian(const Eigen::Vector2f& q);
  void calculateJ_Dot(const Eigen::Vector2f& q, const Eigen::Vector2f& dq);
  void calculateB(const Eigen::Vector2f& q);
  void calculateN(const Eigen::Vector2f& q, const Eigen::Vector2f& dq);
  void updateDynamics(const Eigen::Vector2f& q, const Eigen::Vector2f& dq); 
  
  extern Eigen::Matrix2f J; // Jacobian matrix
  extern Eigen::Matrix2f J_inv; // Jacobian matrix
  extern Eigen::Matrix2f J_dot; // Time derivative of J
  extern Eigen::Matrix2f B; // Intertia matrix
  extern Eigen::Vector2f N; // Nonlinear and coupled dynamics terms
}

#endif
