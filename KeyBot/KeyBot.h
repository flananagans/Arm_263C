#ifndef KEYBOT_H
#define KEYBOT_H
#include "configuration.h"

#include "customtypes.h"

#include <Arduino.h>

// Include ChibiOS RTOS header
#include <ChRt.h>

namespace KeyBot{

void chSetup(void);
extern long t0; // Global start time for all modules

}
#endif
