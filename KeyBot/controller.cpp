#include "controller.h"

#include "eigen.h"

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
  
  float p_arr[2];
  float dp_arr[2];
  Eigen::Map<Eigen::Vector2f> p(&p_arr[0]);
  Eigen::Map<Eigen::Vector2f> dp(&dp_arr[0]);

  // Control variable
  Eigen::Vector2f q_err;
  Eigen::Vector2f q_err_int;
  Eigen::Vector2f p_err;
  Eigen::Vector2f p_err_int;
  float v_arr[2] = {0};
  Eigen::Map<Eigen::Vector2f> v(&v_arr[0]);

  // Gain Matrices
  // JS gains
//  float Kp_arr[2] = {20, 20};
//  float Kd_arr[2] = {5, 5};
//  float Ki_arr[2] = {0.2, 0.2};

  // TS gains
  float Kp_arr[2] = {1200, 1200};
  float Kd_arr[2] = {70, 50};
  float Ki_arr[2] = {7, 7};
  
  Eigen::Matrix2f Kp;
  Eigen::Matrix2f Kd;
  Eigen::Matrix2f Ki;

  // Controller states for saving
  controllerState cState;
  float save_freq = 50;

  /********************** Threads *********************************/
  //Control loop
  static THD_WORKING_AREA(waControl_T, 2048);
  static THD_FUNCTION(Control_T, arg) {
    while(1){
      
      chSysLockFromISR();
      systime_t loop_beg = chVTGetSystemTime();
      
      t = (micros() - KeyBot::t0)/(1e6); // Time in s

      if(!goal_reached) {
        // Update state and copy data to arrays
        Arm::updateState();
        memcpy(&q_arr[0], &Arm::curr_q[0], NUM_JOINTS*sizeof(float));
        memcpy(&dq_arr[0], &Arm::curr_dq[0], NUM_JOINTS*sizeof(float));

        // The controller to run
        simplePID_TS();
      }
      
      // Update the current state
      cState.t = t;
      memcpy(&cState.q[0], &q_arr[0], sizeof(float)*NUM_JOINTS);
      memcpy(&cState.q_des[0], &q_des_arr[0], sizeof(float)*NUM_JOINTS);
      memcpy(&cState.v[0], &v_arr[0], sizeof(float)*NUM_JOINTS);
      
      chSysUnlockFromISR();
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/C_FREQ)));
    }
  }

  // Save state
  static THD_WORKING_AREA(waSaveState_T, 1024);
  static THD_FUNCTION(SaveState_T, arg) {
    while(1){
      systime_t loop_beg = chVTGetSystemTime();
      
      // Send the current state to the SD buffer for saving
      if(SDCARD && !goal_reached) {
        SDCard::cStateBuffer.append(&cState);
      }
      
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/save_freq)));
    }
  }
  /******************** Functions ***************/

  void start() {

    // Initialize gain matrices
    Kp << Kp_arr[0],         0,
                  0, Kp_arr[1];
    Kd << Kd_arr[0],         0,
                  0, Kd_arr[1];
    Ki << Ki_arr[0],         0,
                  0, Ki_arr[1];

    // create tasks
    chThdCreateStatic(waControl_T, sizeof(waControl_T), NORMALPRIO + 3, Control_T, NULL);
    if(SDCARD) {
      chThdCreateStatic(waSaveState_T, sizeof(waSaveState_T), NORMALPRIO + 1, SaveState_T, NULL);
    }
  }

  // Set joint space goal
  void setQGoal(float* q, float* dq, float* ddq) {
    if(Arm::enabled && Kine::fkine(&q[0])) {
      memcpy(&q_des_arr[0], &q[0], NUM_JOINTS*sizeof(float));
      memcpy(&p_des_arr[0], &Kine::last_p[0], 2*sizeof(float)); // x,y
      
      memcpy(&dq_des_arr[0], &dq[0], NUM_JOINTS*sizeof(float));
      memcpy(&ddq_des_arr[0], &ddq[0], NUM_JOINTS*sizeof(float));
      goal_reached = false;
    }
  }

  // Set task space goal
  void setPGoal(float* p) {
    if(Arm::enabled && Kine::ikine(&p[0])) {
      memcpy(&p_des_arr[0], &p[0], 2*sizeof(float)); // x,y
      memcpy(&q_des_arr[0], &Kine::last_q[0], NUM_JOINTS*sizeof(float));  
      goal_reached = false;
    }
  }
  
  /*
   * Simple joint space PID controller
   */
  void simplePID_JS() {

    // Data is accessed using the Eigen::Map vector objects
    q_err = (q_des - q);
    q_err_int = q_err + q_err_int;
    
    if( (dq.norm() > Q_VEL_THRESH) || (q_err.norm() > Q_ERR_THRESH) ) {
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

  /*
   * Simple task space PID controller
   */
  void simplePID_TS() {

    // Convert joint angles to task space
    if(!Kine::fkine(&q_arr[0])) {
      Serial.println("Inverse kinematics failed :(");
      return;
    }
    memcpy(&p_arr[0], &Kine::last_p[0], 2*sizeof(float));

    // Convert joint velocities to task space
    Dyn::calculateJacobian(q);
    dp = Dyn::J * dq;
    dp_des = Dyn::J * dq_des;
  
    // Data is accessed using the Eigen::Map vector objects
    p_err = (p_des - p);
    p_err_int = p_err + p_err_int;
        
    if( (dp.norm() > P_VEL_THRESH) || (p_err.norm() > P_ERR_THRESH) ) {
      v = Kp*p_err + Kd*(dp_des - dp) + Ki*p_err_int;

      // Convert back to joint space
      v = Dyn::J.transpose() * v;
      
      Arm::setV(&v_arr[0]);
    } else { // Goal is reached, stop the arm and press the key
      v_arr[0] = 0;
      v_arr[1] = 0;
      Arm::setV(&v_arr[0]);
      
      p_err_int(0) = 0;
      p_err_int(1) = 0;
      goal_reached = true;
      if(Traj::traj_finished) {
        Arm::pressKey();
      }
    }
  }

  /*
   * Inverse dynamics task space controller
   */
  void ID_TS() {

    // Convert joint angles to task space
    if(!Kine::fkine(&q_arr[0])) {
      Serial.println("Inverse kinematics failed :(");
      return;
    }
    memcpy(&p_arr[0], &Kine::last_p[0], 2*sizeof(float));

    // Update the dynamics model
    Dyn::updateDynamics(q, dq); 

    // Convert joint velocities to task space
    dp = Dyn::J * dq;
    dp_des = Dyn::J * dq_des;
    ddp_des = Dyn::J * ddq_des + Dyn::J_dot * dq_des;
  
    // Data is accessed using the Eigen::Map vector objects
    p_err = (p_des - p);
        
    if( (dp.norm() > P_VEL_THRESH) || (p_err.norm() > P_ERR_THRESH) ) {
      Eigen::Vector2f y = Dyn::J_inv * (ddp_des + Kd*(dp_des - dp) + Kp*p_err - Dyn::J_dot*dq);

      v = Dyn::B*y + Dyn::N;
      
      Arm::setV(&v_arr[0]);
    } else { // Goal is reached, stop the arm and press the key
      v_arr[0] = 0;
      v_arr[1] = 0;
      Arm::setV(&v_arr[0]);
      
      p_err_int(0) = 0;
      p_err_int(1) = 0;
      goal_reached = true;
      if(Traj::traj_finished) {
        Arm::pressKey();
      }
    }
  }
}
