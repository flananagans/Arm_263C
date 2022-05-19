#include "controller.h"

#include "eigen.h"
#include "trajectory.h"

namespace Controller {

  volatile bool goal_reached = true;

  float t = 0;

  // Desired task space values
  float p_des_arr[2];
  float dp_des_arr[2];
  float ddp_des_arr[2];
  Eigen::Map<Eigen::Vector2f> p_des(&p_des_arr[0]);
  Eigen::Map<Eigen::Vector2f> dp_des(&dp_des_arr[0]);
  Eigen::Map<Eigen::Vector2f> ddp_des(&dp_des_arr[0]);

  // Desired joint space values
  float q_des_arr[2];
  float dq_des_arr[2];
  float ddq_des_arr[2];
  Eigen::Map<Eigen::Vector2f> q_des(&q_des_arr[0]);
  Eigen::Map<Eigen::Vector2f> dq_des(&dq_des_arr[0]);
  Eigen::Map<Eigen::Vector2f> ddq_des(&dq_des_arr[0]);

  // Current values
  float q_arr[2];
  float dq_arr[2];
  Eigen::Map<Eigen::Vector2f> q(&q_arr[0]);
  Eigen::Map<Eigen::Vector2f> dq(&dq_arr[0]);

  // Control variable
  Eigen::Vector2f q_err;
  Eigen::Vector2f q_err_int;
  float v_arr[2] = {0};
  Eigen::Map<Eigen::Vector2f> v(&v_arr[0]);

  // Gain Matrices
  float Kp_arr[2] = {5, 5};
  float Kd_arr[2] = {1, 1};
  float Ki_arr[2] = {1, 1};
  Eigen::Matrix2f Kp;
  Eigen::Matrix2f Kd;
  Eigen::Matrix2f Ki;

  /********************** Threads *********************************/
  //Control loop
  static THD_WORKING_AREA(waControl_T, 2048);
  static THD_FUNCTION(Control_T, arg) {
    while(1){
      
      chSysLockFromISR();
      systime_t loop_beg = chVTGetSystemTime();
      
      t = (micros() - KeyBot::t0)/(1e6); // Time in s
      
      simplePD_JS();
      
      // Send the current state to the SD buffer for saving
      if(SDCARD && !goal_reached) {
        controllerState cState;
        cState.t = t;
        memcpy(&cState.q[0], &q_arr[0], sizeof(float)*NUM_JOINTS);
        memcpy(&cState.q_des[0], &q_des_arr[0], sizeof(float)*NUM_JOINTS);
        memcpy(&cState.v[0], &v_arr[0], sizeof(float)*NUM_JOINTS);
        SDCard::cStateBuffer.append(&cState);
      }
      
      chSysUnlockFromISR();
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
    Ki << Ki_arr[0],         0,
                  0, Ki_arr[1];

    // create task at priority two
    chThdCreateStatic(waControl_T, sizeof(waControl_T), NORMALPRIO + 2, Control_T, NULL);
  }

  // Set joint space goal
  void setQGoal(float* q) {
    if(Kine::fkine(&q[0])) {
      memcpy(&q_des_arr[0], &q[0], NUM_JOINTS*sizeof(float));
      memcpy(&p_des_arr[0], &Kine::last_p[0], 2*sizeof(float)); // x,y
      goal_reached = false;
    }
  }

  // Set task space goal
  void setPGoal(float* p) {
    if(Kine::ikine(&p[0])) {
      memcpy(&p_des_arr[0], &p[0], 2*sizeof(float)); // x,y
      memcpy(&q_des_arr[0], &Kine::last_q[0], NUM_JOINTS*sizeof(float));  
      goal_reached = false;
    }
  }
  
  /*
   * Simple joint space PD controller
   */
  void simplePD_JS() {
    if(!goal_reached) {
      // Update state and copy data to arrays
      Arm::updateState();
      memcpy(&q_arr[0], &Arm::curr_q[0], 2*sizeof(float));
      memcpy(&dq_arr[0], &Arm::curr_dq[0], 2*sizeof(float));

      Serial.print(q(0));
      Serial.print(",");
      Serial.println(q(1));

      // Data is accessed using the Eigen::Map vector objects
      q_err = (q_des - q);
      q_err_int = q_err + q_err_int; 
      if( (dq.norm() > VEL_THRESH) || (q_err.norm() > ERR_THRESH) ) {
        v = Kp*q_err + Kd*(dq_des - dq) + Ki*q_err_int;
        Arm::setV(&v_arr[0]);
      } else { // Goal is reached, stop the arm and press the key
        v_arr[0] = 0;
        v_arr[1] = 0;
        Arm::setV(&v_arr[0]);
        
        q_err_int(0) = 0; 
        q_err_int(1) = 0;
        goal_reached = true;
        if(Traj::traj_finished) {
          Arm::pressKey();
        }
      }
    }
  }
}
