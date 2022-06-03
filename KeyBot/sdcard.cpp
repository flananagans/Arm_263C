#include "sdcard.h"

namespace SDCard {

  float SD_FREQ = 10; // Hz

  // Files and names
  String cStateFilename;
  static File cStateFile;

  // Data buffers
  HopBuf<controllerState_t> cStateBuffer;

  char * DEFAULT_TRIALNAME = (char*)"WF14";
  char * trialname = DEFAULT_TRIALNAME;

  // Configuration variables
  const int SD_CS = BUILTIN_SDCARD;    
  thread_t *writeFiles = NULL;

  //Thread for periodically writing on the sdcard
  static THD_WORKING_AREA(waWriteFiles_T, 8192);
  static THD_FUNCTION(WriteFiles_T, arg) {  
    while(true){
      systime_t loop_beg = chVTGetSystemTime();

      if(cStateBuffer.missed > 0) {
        Serial.println(cStateBuffer.missed);
      }
      saveControllerState();
      
      // Loop at constant frequency
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/SD_FREQ)));
    }
  }

  /*
   * Initialize SD card
   */
  bool initSD(void){
    if (!SD.begin(SD_CS)) {
      return false;
    } else { 
      return true;
    }
  }

  bool createFiles(void){
    
    // Create the files
    cStateFilename = String("cState/") + String(trialname) + String(".bin"); 
    if (SD.exists(cStateFilename.c_str())) {
      Serial.println("File exists... overwriting");
      SD.remove(cStateFilename.c_str());
    }
    
    // Controller State 
    SD.mkdir("cState");  
    cStateFile = SD.open(cStateFilename.c_str(), O_WRITE | O_CREAT); 
    if(!cStateFile){
      Serial.print("Could not open ");
      Serial.println(cStateFilename);
      return false;
    }
    cStateFile.close();
    
    return true;
  }

  /*
   * Close all files
   */
  bool closeFiles() {
    if(cStateFile) {
      cStateFile.close();
      Serial.print("Missed points: ");
      Serial.println(cStateBuffer.missed);
    }
  }

  /*
   * Startup function
   */
  void start() {

    if(initSD()) {
      createFiles();
    
      // create tasks at priority one
      writeFiles = chThdCreateStatic(waWriteFiles_T, sizeof(waWriteFiles_T), NORMALPRIO + 1, WriteFiles_T, NULL);
    } else {
      Serial.println("SD FAILED");
    }
  }

  /* 
   * Save the controller state to SD Card
   */
  void saveControllerState() {
    
    controllerState_t curr_cState;
    if( (cStateBuffer.len > 0) && !cStateFile) {
      Serial.println("Opened file");
      cStateFile = SD.open(cStateFilename.c_str(), O_WRITE);
    }
    while(cStateBuffer.len > 0) {
      // Stop all interrupts while we fetch data
      chSysLockFromISR();
        cStateBuffer.pop(&curr_cState);
      chSysUnlockFromISR();
      // Write to file
      cStateFile.write((uint8_t*)&curr_cState, sizeof(controllerState_t));
    }
    
    if(cStateFile && Controller::goal_reached) { // close file once the goal is reached
      cStateFile.close(); 
      Serial.print("Missed points: ");
      Serial.println(cStateBuffer.missed);
    }
  }
}
