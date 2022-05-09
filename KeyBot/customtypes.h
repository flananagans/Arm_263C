#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types for data and other information we need to store and move around
*/

typedef struct controllerState{
  float t;
  float q[NUM_JOINTS];
  float q_des[NUM_JOINTS];  
  float v[NUM_JOINTS];
} controllerState_t;

#endif
