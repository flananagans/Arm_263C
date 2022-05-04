#ifndef SERIALCOMMAND_H
#define SERIALCOMMAND_H

#include <Arduino.h>

#include <SerialCommand.h>  // Due to the way the Arduino IDE compiles

namespace SerialCom{

void start(void);

extern long startTime;

// Callback functions for specific serial commands
void INFO(void);
void disableArm(void);
void enableArm(void);

void unrecognized();
void unrecognized(const char *command);

}

#endif
