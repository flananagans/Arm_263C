#include "dynamics.h"

namespace Dyn {

  Eigen::Matrix2f J; // Jacobian matrix
  Eigen::Matrix2f J_T; // Jacobian transpose matrix

  /*
   * Calculate the Jacobian for the given joint angles
   *  The Jacobian matrix is stored in Dyn::J
   */
  void calculateJacobian(float* q) {
    J(0,0) = -1*LINK_1_L*sin(q[0]) - LINK_2_L*sin(q[0] + q[1]);
    J(0,1) = -1*LINK_2_L*sin(q[0] + q[1]);
    J(1,0) = LINK_1_L*cos(q[0]) + LINK_2_L*cos(q[0] + q[1]);
    J(1,1) = LINK_2_L*cos(q[0] + q[1]);

    J_T = J.transpose();
  }
}
