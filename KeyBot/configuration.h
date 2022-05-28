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
// CONTROL CONFIGURATION
//
#define C_FREQ 500
#define Q_ERR_THRESH 0.05f
#define Q_VEL_THRESH 0.05f
#define P_ERR_THRESH 0.002f
#define P_VEL_THRESH 0.002f

/////////////////////////////////////////////////////
//
// ARM CONFIGURATION
//
#define PORT_ARM Serial1
#define PIN_EE 4 // End-effector servo pin
#define NUM_JOINTS 2
// Joint lengths
#define LINK_1_L 0.293f
#define LINK_2_L 0.211f
// Joint offsets
#define OFFSET_Q1 0.11f
#define OFFSET_Q2 0.30f
// Joint limits
#define J1_MINANG -1.5708f
#define J1_MAXANG 1.5708f
#define J2_MINANG 0.0f
#define J2_MAXANG 2.95f
#define EPS 1e-10f
#define J_MAXACC 10.0f
#define EE_ATTACHED true

// FIRMWARE INFO STRING
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_INFO "KeyBot v0.1" TOSTRING(FIRMWARE_VERSION)

// Enable SD Card
#define SDCARD true

#endif
