#include "arm.h"
#include <Servo.h>

/*
 * Interface for the SerialArm object
 *  Get the current state of the joints
 *  Set the desired voltage of each joint
 */
namespace Arm {
  
  SerialArm arm(&PORT_ARM, 36);
  Servo ee; // end effector servo
  binary_semaphore_t ee_bsem;
  
  float V_MAX = 12.0;

  // Current position
  float curr_q[2] = {0};
  // Current velocity
  float curr_dq[2] = {0};

  int KEY_PRESS = 30;
  int KEY_REL = 100;

  static THD_WORKING_AREA(waPressKey_T, 64);
  static THD_FUNCTION(PressKey_T, arg) {
    
    while (true) {
      chBSemWait(&ee_bsem);
      
      ee.write(KEY_PRESS); // press key
      Serial.println("Key Pressed!");
      chThdSleep(TIME_MS2I(400));
      ee.write(KEY_REL); // release key
      Serial.println("Key Released!");
    }
  }

  void start() {

    // Set up the dynamixels on the arm
    for(uint8_t j = 0; j < NUM_JOINTS; j++) {
      arm.addJoint(j + 1);
    }  
    arm.start();
    // Configure with delay of 0us (2*var), PWM mode, max voltage 
    arm.configure(0, 16, V_MAX);
    disable();

    if(EE_ATTACHED) {
      // Set up the servo on the end effector
      ee.attach(PIN_EE);
  
      // Initialize the end effector semaphore
      chBSemObjectInit(&ee_bsem, true);
      // Start thread
      chThdCreateStatic(waPressKey_T, sizeof(waPressKey_T),
                      NORMALPRIO + 1, PressKey_T, NULL);
  
      ee.write(KEY_REL);
    }
  }

  void enable() {
    arm.enable();
  }

  void disable() {
    arm.disable();
  }

  /*
   * Update position and velocity of the arm
   *  Access values through two extern arrays
   */
  bool updateState() {
    arm.getAllPos(&curr_q[0]);
    curr_q[0] = curr_q[0] - OFFSET_Q1;
    curr_q[1] = curr_q[1] - OFFSET_Q2;
    arm.getAllVel(&curr_dq[0]);

    if(isnan(curr_q[0]) || isnan(curr_dq[0])) {
      return false;
    }
    return true;
  }

  /*
   * Set the voltage of the motors
   *  v_d should be a pointer to a 1 x arm.numJoints float array 
   */
  void setV(float* v_d) {
    arm.setV(v_d);
  }

  /*
   * Press the key
   */
  void pressKey() {
    if(EE_ATTACHED) {
      chSysLockFromISR();
      chBSemSignalI(&ee_bsem);
      chSysUnlockFromISR();
    }
  }
}
