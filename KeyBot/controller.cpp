#include "controller.h"

#include "eigen.h"

namespace Controller {

  bool goal_reached = true;

  // Desired values
  float q_des_arr[2] = {0};
  float dq_des_arr[2] = {0};
  float ddq_des_arr[2] = {0};
  Eigen::Map<Eigen::Vector2f> q_des(&q_des_arr[0], 2);
  Eigen::Map<Eigen::Vector2f> dq_des(&dq_des_arr[0], 2);
  Eigen::Map<Eigen::Vector2f> ddq_des(&dq_des_arr[0], 2);

  // Current values
  float q_arr[2];
  float dq_arr[2];
  Eigen::Map<Eigen::Vector2f> q(&q_arr[0], 2);
  Eigen::Map<Eigen::Vector2f> dq(&dq_arr[0], 2);

  // Control variable
  Eigen::Vector2f q_err;
  float v_arr[2] = {0};
  Eigen::Map<Eigen::Vector2f> v(&v_arr[0], 2);

  // Gain Matrices
  float Kp_arr[2] = {5,5};
  float Kd_arr[2] = {0.5, 0.5};
  Eigen::Matrix2f Kp;
  Eigen::Matrix2f Kd;

  /********************** Threads *********************************/
  //Control loop
  static THD_WORKING_AREA(waControl_T, 2048);
  static THD_FUNCTION(Control_T, arg) {
    while(1){
      systime_t loop_beg = chVTGetSystemTime();
      
      simplePD_JS();
      
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/C_FREQ)));
    }
  }

  void start() {

    // Initialize gain matrices
    Kp << Kp_arr[0],         0,
                  0, Kp_arr[1];
    Kd << Kd_arr[0],         0,
                  0, Kd_arr[1];

    // create task at priority two
    chThdCreateStatic(waControl_T, sizeof(waControl_T), NORMALPRIO + 2, Control_T, NULL);
  }

  void setQGoal(float* q) {
    memcpy(&q_des[0], q, NUM_JOINTS*sizeof(float));
    goal_reached = false;
  }
  
  /*
   * Simple joint space PD controller
   */
  void simplePD_JS() {
    if(!goal_reached) {
      Arm::updateState();
      float q_arr[2];
      memcpy(&q_arr[0], &Arm::curr_q[0], 2*sizeof(float));
      memcpy(&dq_arr[0], &Arm::curr_dq[0], 2*sizeof(float));
  
      q_err = (q_des - q);
      Serial.print("q: ");
      Serial.print(q(0));
      Serial.print(",");
      Serial.println(q(1));
      
      if(q_err.norm() > ERR_THRESH) {
        
        Serial.println("hola_c");
        
        v = Kp*q_err - Kd*(dq_des - dq);
        Arm::setV(&v_arr[0]);
      } else {
        goal_reached = true;
      }
    }
  }
}
