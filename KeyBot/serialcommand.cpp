#include "KeyBot.h"
#include "serialcommand.h"

#include "arm.h"

char strbuffer[STRBUFFERSIZE];

namespace SerialCom{

  SerialCommand sCmd;

  thread_t *readSerial = NULL;

  int freq = 10;
  long startTime = 0;

  /********************** Threads *********************************/
  //To process //Serial commands
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
