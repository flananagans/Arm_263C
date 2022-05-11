#ifndef KEYBOT_H
#define KEYBOT_H
#include "configuration.h"

#include "customtypes.h"

#include <Arduino.h>

// Include ChibiOS RTOS header
#include <ChRt.h>

namespace KeyBot{

void startComs(void);
void chSetup(void);

extern const char firmwareInfo[];
extern char strbuffer[STRBUFFERSIZE];   // Buffer for str data for use across all files

extern long t0; // Global start time for all modules

}
#endif
