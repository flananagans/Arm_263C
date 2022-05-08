#include "KeyBot.h"
#include "kinematics.h"

/*
 * Forward and inverse kinematics calculations for the arm 
 */
namespace Kine {

  float last_q[NUM_JOINTS] = {0};
  float last_p[2] = {0};

  /*
   * Forward kinematics
   */
  bool fkine(float* q) {
    last_p[0] = LINK_1_L*cos(q[0]) + LINK_2_L*cos(q[0] + q[1]);
    last_p[1] = LINK_1_L*sin(q[0]) + LINK_2_L*sin(q[0] + q[1]);
    return true;
  }

  /* 
   * Inverse kinematics
   */
  bool ikine(float* p) {

    // Check workspace
    float r2 = pow(p[0], 2) + pow(p[1], 2); // length to goal squared
    if(sqrt(r2) > LINK_1_L + LINK_2_L + EPS) {
        return false;
    }

    float a1 = LINK_1_L;
    float a2 = LINK_2_L;
    float a1_sq = pow(a1, 2);
    float a2_sq = pow(a2,2);

    // Calculating theta2
    float c2 = (r2 - (a1_sq + a2_sq)) / (2*a1*a2);
    if(c2 > 1 - EPS) {
      last_q[1] = 0;
    } else {
      last_q[1] = atan2(sqrt(1 - pow(c2, 2)), c2);
    }
    if(last_q < 0) {
      last_q[1] = atan2(-1*sqrt(1 - pow(c2, 2)), c2); //swap to other config
    }
    // Check limits
    if( (last_q[1] > J2_MAXANG) || (last_q[1]< J2_MINANG) ) {
        return false;
    }
    
    // Calculate theta1
    float beta = atan2(p[1], p[0]);
    float psi = acos( (r2 + a1_sq - a2_sq) / (2*a1*sqrt(r2)) );
    last_q[0] = beta - psi;
    // Check limits
    if( (last_q[0] > J1_MAXANG) || (last_q[0] < J1_MINANG) ) {
        return false;
    }
    
    return true;
  }

}
