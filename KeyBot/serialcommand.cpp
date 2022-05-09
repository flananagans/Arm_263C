#include "serialcommand.h"

char strbuffer[STRBUFFERSIZE];

namespace SerialCom {

  SerialCommand sCmd;

  thread_t *readSerial = NULL;

  float freq = 10.0;
  long startTime = 0;

  /********************** Threads *********************************/
  //To process Serial commands
  static THD_WORKING_AREA(waReadSerial_T, 1024);
	static THD_FUNCTION(ReadSerial_T, arg) {
    while(1){
      systime_t loop_beg = chVTGetSystemTime();
      
		  sCmd.readSerial();
      
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/freq)));
    }
	}
 
  /***************************************************************/

	void start(void){

    //Command dictionary. Links the target command string to the function name
    sCmd.addCommand("INFO", INFO);  // Display information about the firmware
    sCmd.addCommand("DIS", disableArm);
    sCmd.addCommand("EN", enableArm);
    sCmd.addCommand("S_Q", setQ); // set joint angles
    sCmd.addCommand("S_V", setV); // set motor voltages
    sCmd.addCommand("FK", testFK);
    sCmd.addCommand("IK", testIK);
    sCmd.addCommand("S_K", setKey); // set desired key
    
    sCmd.setDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")
    Serial.println("Serial Commands are ready");

    // create task at priority one
    chThdCreateStatic(waReadSerial_T, sizeof(waReadSerial_T), NORMALPRIO + 1, ReadSerial_T, NULL);
	}

void INFO() {
  char info[50]; 
  int num = sprintf(info, "Firmware: %s", FIRMWARE_INFO); 
  Serial.println(info);
}

void disableArm() {
  Arm::disable();
}

void enableArm() {
  Arm::enable();
}

/*
 * Set desired joint angles
 */
void setQ() {
  
  float q[2] = {0};
  // Collect additional arguments
  char* arg;
  arg = sCmd.next();
  if(arg != NULL) {
    q[0] = atof(arg);  
  }
  arg = sCmd.next();
  if(arg != NULL) {
    q[1] = atof(arg); 
  }

  Controller::setQGoal(&q[0]);
}

/*
 * Set voltage of the motors
 */
void setV() {
  
  float v[2] = {0};
  // Collect additional arguments
  char* arg;
  arg = sCmd.next();
  if(arg != NULL) {
    v[0] = atof(arg);  
  }
  arg = sCmd.next();
  if(arg != NULL) {
    v[1] = atof(arg); 
  }

  Arm::setV(&v[0]);
}

/*
 * Function for testing forward kinematics
 */
void testFK() {

  float q[2] = {0};
  // Collect additional arguments
  char* arg;
  arg = sCmd.next();
  if(arg != NULL) {
    q[0] = atof(arg);  
  }
  arg = sCmd.next();
  if(arg != NULL) {
    q[1] = atof(arg); 
  }

  if(Kine::fkine(&q[0])) {
    Serial.print(Kine::last_p[0], 3);
    Serial.print(",");
    Serial.println(Kine::last_p[1], 3);
  } else {
    Serial.println("FKINE FAILED");
  }
}

/*
 * Function for testing inverse kinematics
 */
void testIK() {

  float p[2] = {0};
  // Collect additional arguments
  char* arg;
  arg = sCmd.next();
  if(arg != NULL) {
    p[0] = atof(arg);  
  }
  arg = sCmd.next();
  if(arg != NULL) {
    p[1] = atof(arg);  
  }

  if(Kine::ikine(&p[0])) {
    Serial.print(Kine::last_q[0], 3);
    Serial.print(",");
    Serial.println(Kine::last_q[1], 3);
  } else {
    Serial.println("IKINE FAILED");
  }
}

/*
 * Set the desired key to press
 */
void setKey() {
  float pos[2] = {0};
  // Collect additional arguments
  char* arg;
  arg = sCmd.next();
  if(arg != NULL) {
    Traj::getCoord(&pos[0], *arg);  
  }
  Serial.print(pos[0], 3);
  Serial.print(",");
  Serial.println(pos[1], 3);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.print("Unknown Command: ");
  Serial.print(command);

  // Parsing through arguments (arguments are delimited by spaces in the char array)
  char *arg;
  arg = sCmd.next();
  while(arg != NULL) {
    Serial.print(' ');
    Serial.print(arg);
    arg = sCmd.next();
  }
  Serial.println(' ');

}

}
