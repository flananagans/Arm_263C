#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <stdint.h>
/* 
Definition of custom types for data and other information we need to store and move around
*/

typedef struct floatSample{
  float timestamp;
  float value;    
} floatSample_t;

#endif
