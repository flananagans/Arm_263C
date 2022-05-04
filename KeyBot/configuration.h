#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define FIRMWARE_VERSION "v0.1" 

/* Global configuration such as pins, rates etc */

/***********************************************/
////////////////////////////////////////////////
/// LED 
/// (classic arduino led)
/// 
#ifndef PIN_LED
#define PIN_LED 13   //(Already defined in Arduino.h)
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define SDBUFFERSIZE 512 // Size of storage buffer for SD data
#define STRBUFFERSIZE 64

/////////////////////////////////////////////////////
//
// ARM CONFIGURATION
//
#define PORT_ARM Serial1

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO "KeyBot v0.1" TOSTRING(FIRMWARE_VERSION)

// Enable SD Card
#define SDCARD false

#endif
