#include "KeyBot.h"
#include "arm.h"
#include "controller.h"
#include "serialcommand.h"
#include "sdcard.h"

namespace KeyBot {
  
  long t0 = 0;
  
  //------------------------------------------------------------------------------
  // Pulse thread, toggles the LED as a heartbeat.
  // 64 byte stack beyond task switch and interrupt needs.
  static THD_WORKING_AREA(waPulse_T, 64);
  
  static THD_FUNCTION(Pulse_T, arg) {
    (void)arg;
    pinMode(PIN_LED, OUTPUT);
    while (1) {
      systime_t loop_beg = chVTGetSystemTime();
      
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      
      // Loop at constant frequency (2 Hz)
      chThdSleepUntil(chTimeAddX(loop_beg, TIME_US2I(1e6/2)));
    }
  }
  
  void startComs() {
    
    Serial.begin(115200);
    while(!Serial) {};
    
  }
  
  //------------------------------------------------------------------------------
  // continue setup() after chBegin().
  void chSetup() {
    // Start pulse thread
    chThdCreateStatic(waPulse_T, sizeof(waPulse_T), NORMALPRIO + 1, Pulse_T, NULL);
    t0 = micros();
    Serial.println("Hello, This is KeyBot!");
    
    // Start selected programs
    Arm::start();
    SerialCom::start();
    Controller::start();
  }
}

// Out of namespace KeyBot, normal Arduino startup
//------------------------------------------------------------------------------
void setup() {
  // Start all communications
  KeyBot::startComs();

  // Initialize OS and then call chSetup.
  chBegin(KeyBot::chSetup);
  while (true) {} // blocks here and Chibi takes over
}
//------------------------------------------------------------------------------
// loop() is the main thread.  Not used with ChibiOS
void loop() {
}
