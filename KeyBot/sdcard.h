#ifndef SDCARD_H
#define SDCARD_H

#include "KeyBot.h"
#include "controller.h"

#include "HopBuf.h"
#include <SD.h>

namespace SDCard {

  void start(void);
  bool createFiles(void);
  void saveControllerState(void);
  bool initSD(void);
  bool createFiles(void);

  extern HopBuf<controllerState_t> cStateBuffer;
}

#endif
