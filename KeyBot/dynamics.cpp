#include "dynamics.h"

namespace Dyn {

  Eigen::Matrix2f J; // Jacobian matrix
  Eigen::Matrix2f J_inv; // Inverse of Jacobian matrix
  Eigen::Matrix2f J_dot; // Time derivative of J
  Eigen::Matrix2f B; // Inertia matrix
  Eigen::Vector2f N; // Nonlinear and coupled dynamics terms

  const float k_r = 193;
  Eigen::Matrix2f F { {k_r*(0.00692)*k_r, 0},
                      {0, k_r*(0.00416)*k_r} }; // Viscous friction

  /*
   * Calculate the Jacobian for the given joint angles
   *  The Jacobian matrix is stored in Dyn::J
   */
  void calculateJacobian(const Eigen::Vector2f& q) {

    float s1 = sin(q(0));
    float c1 = cos(q(0));
    float s12 = sin(q(0) + q(1));
    float c12 = cos(q(0) + q(1));
    
    J(0,0) = -1*LINK_1_L*s1 - LINK_2_L*s12;
    J(0,1) = -1*LINK_2_L*s12;
    J(1,0) = LINK_1_L*c1 + LINK_2_L*c12;
    J(1,1) = LINK_2_L*c12;
  }

  /*
   * Calculate the time derivative of the Jacobian matrix
   */
  void calculateJ_Dot(const Eigen::Vector2f& q, const Eigen::Vector2f& dq) {

    float s1 = sin(q(0));
    float c1 = cos(q(0));
    float s12 = sin(q(0) + q(1));
    float c12 = cos(q(0) + q(1));
    
    J_dot(0,0) = -1*LINK_1_L*c1*dq(0) - LINK_2_L*c12*(dq(0) + dq(1));
    J_dot(0,1) = -1*LINK_2_L*c12*(dq(0) + dq(1));
    J_dot(1,0) = -1*LINK_1_L*s1*dq(0) - LINK_2_L*s12*(dq(0) + dq(1));
    J_dot(1,1) = -1*LINK_2_L*s12*(dq(0) + dq(1));
  }

  /*
   * Calculate the current inertia matrix
   */
  void calculateB(const Eigen::Vector2f& q) {
    B(0,0) = 0.01388*cos(q(1)) + 249.5876;
    B(0,1) = 0.006942*cos(q(1)) + 0.00449;
    B(1,0) = B(0,1);
    B(1,1) = 283.0969;
  }

  /*
   * Calculate the current coupled and nonlinear terms
   */
  void calculateN(const Eigen::Vector2f& q, const Eigen::Vector2f& dq) {
    Eigen::Matrix2f C;
    C(0,0) = -0.006942*sin(q(1))*dq(1);
    C(0,1) = -0.006942*sin(q(1))*(dq(0) + dq(1));
    C(1,0) = 0.006942*sin(q(1))*dq(0);
    C(1,1) = 0;
    
    N = (F + C)*dq;
  }

  /*
   * Update the dynamical model
   */
  void updateDynamics(const Eigen::Vector2f& q, const Eigen::Vector2f& dq) {

    calculateJacobian(q);
    J_inv = J.inverse();
    calculateJ_Dot(q, dq);
    calculateB(q);
    calculateN(q, dq);
  }
}
