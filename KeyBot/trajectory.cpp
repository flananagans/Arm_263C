#include "trajectory.h"

namespace Traj {

  float dur = 0; // total duration of current trajectory in sec
  float accs[NUM_JOINTS] = {0}; // Desired accelerations for each joint
  float q_i[NUM_JOINTS] = {0}; // initial joint angles of the trajectory
  float q_f[NUM_JOINTS] = {0}; // final joint angles of the trajectory
  long t_start = 0; // Start time of the trajectory
  float t = 0;
  volatile bool traj_finished = true;
  float traj_freq = 10; // Hz

  /********************** Threads *********************************/
  //Thread to send desired trajectory points to controller
  static THD_WORKING_AREA(waTraj_T, 512);
  static THD_FUNCTION(Traj_T, arg) {
    while(1){

      systime_t loop_beg = chVTGetSystemTime();

      t = (micros() - t_start)/(1e6); // Current duration of trajectory in sec
      if(t < dur) {
        float q_d[NUM_JOINTS]; // desired joint angles
        float dq_d[NUM_JOINTS]; // desired joint velocities
        float ddq_d[NUM_JOINTS]; // desired joint accels
        
        float dir = 1; // direction of accel (1 or -1)
        float t_offset = 0; // offset in time (0 or dur)
        float* q_offset = &q_i[0]; // offset in position (q_i or q_f)
        
        if(t > dur/2) {
          // We are past halfway, so opposite acceleration and calculate backwards from q_f
          dir = -1;
          t_offset = dur;
          q_offset = &q_f[0];
        }
        
        // Calculate desired joint angles and derivatives for the current time
        for(uint8_t j = 0; j < NUM_JOINTS; j ++) {
          ddq_d[j] = dir*accs[j];
          q_d[j] = 0.5*ddq_d[j]*pow(t - t_offset, 2) + q_offset[j];
          dq_d[j] = ddq_d[j]*(t - t_offset);
        }

        // Send goal to controller
        Controller::setQGoal(&q_d[0]);
        
      }else if(!traj_finished){
        // Send final goal to controller
        Controller::setQGoal(&q_f[0]);
        traj_finished = true;
      }
      
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/traj_freq)));
    }
  }

  void start() {
    // Start thread
    chThdCreateStatic(waTraj_T, sizeof(waTraj_T), NORMALPRIO + 1, Traj_T, NULL);
  }

  /*
   * Get the x y coordinates for the letter
   *  Coordinates written to 'pos' array
   */
  bool getCoord(float* pos, char letter) {
    if(letter == '1') { // Space bar is mapped to the character '1'
      memcpy(&pos[0], &keyCoords[26][0], 2*sizeof(float));
    } else if( letter > 64 && letter < 91) { // Use ASCII code to index
      memcpy(&pos[0], &keyCoords[(uint8_t)letter - 65][0], 2*sizeof(float));
    } else {
      return false;
    }
    // Convert units
    pos[0] = 0.0254*pos[0];
    pos[1] = 0.0254*pos[1];
    return true;
  }

  void setKey(char letter) {
    float p[2] = {0};
    if(getCoord(&p[0], letter)) {
      Controller::setPGoal(&p[0]);
    }
  }

  /*
   * Generate a trajectory in joint space
   */
  void generateTrajectory(char letter) {

    // Get current joint angles
    if(!Arm::updateState()) {
      Serial.println("Updating state failed :(");
      return;     
    }
    memcpy(&q_i[0], &Arm::curr_q[0], NUM_JOINTS*sizeof(float));
    
    // Get final x y coordinates for the letter
    float p_f[2] = {0}; // final x y coordinates
    if(!getCoord(&p_f[0], letter)) {
      Serial.println("Getting coordinate failed :(");
      return;
    }

    // Get final joint angles
    if(!Kine::ikine(&p_f[0])) {
      Serial.println("Inverse kinematics failed :(");
      return;
    }
    memcpy(&q_f[0], &Kine::last_q[0], NUM_JOINTS*sizeof(float));

    // Get the limiting time (assuming all joints have same max acc)
    float del_q_max = 0; 
    float del_q_j = 0;
    for(uint8_t j = 0; j < NUM_JOINTS; j++) {
      del_q_j = fabs(q_f[j] - q_i[j]);
      del_q_max = (del_q_j > del_q_max) ? del_q_j : del_q_max;
    }
    dur = 2*sqrt(del_q_max/J_MAXACC);

    // Calculate desired accelerations for each joint
    for(uint8_t j = 0; j < NUM_JOINTS; j++) {
      accs[j] = (q_f[j] - q_i[j]) / pow(dur/2, 2);
    }
    
    // Reset start time
    traj_finished = false;
    t_start = micros();
  }
  
}
