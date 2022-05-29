#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "KeyBot.h"
#include "controller.h"

namespace Traj {

  void start(void);
  bool getCoord(float* pos, char letter);
  void setKey(char letter);
  void generateTrajectory(char letter);
  extern volatile bool traj_finished;
  
  const float keyCoords[29][2] = {0.1810,  0.1032,
                                  0.1619,  0.0175,
                                  0.1619,  0.0556,
                                  0.1810,  0.0651,
                                  0.2000,  0.0706,
                                  0.1810,  0.0460,
                                  0.1810,  0.0270,
                                  0.1810,  0.0079,
                                  0.2000, -0.0246,
                                  0.1810, -0.0111,
                                  0.1810, -0.0302,
                                  0.1810, -0.0492,
                                  0.1619, -0.0206,
                                  0.1619, -0.0016,
                                  0.2000, -0.0437,
                                  0.2000, -0.0627,
                                  0.2000,  0.1087,
                                  0.2000,  0.0516,
                                  0.1810,  0.0841,
                                  0.2000,  0.0325,
                                  0.2000, -0.0056,
                                  0.1619,  0.0365,
                                  0.2000,  0.0897,
                                  0.1619,  0.0746,
                                  0.2000,  0.0135,
                                  0.1619,  0.0937,
                                  0.1397,  0.0151,
                                  0.2191, -0.1124,
                                       0,       0}; // (x, y) coords for each character IN METERS
}

#endif
