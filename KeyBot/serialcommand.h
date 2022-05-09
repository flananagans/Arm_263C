#ifndef SERIALCOMMAND_H
#define SERIALCOMMAND_H

#include <Arduino.h>
#include <SerialCommand.h> 

#include "KeyBot.h"
#include "arm.h"
#include "kinematics.h"
#include "controller.h"
#include "trajectory.h"

namespace SerialCom {

void start(void);

extern long startTime;

// Callback functions for specific serial commands
void INFO(void);
void disableArm(void);
void enableArm(void);
void setQ(void);
void setV(void);
void testFK(void);
void testIK(void);
void setKey(void);

void unrecognized();
void unrecognized(const char *command);

}

#endif
