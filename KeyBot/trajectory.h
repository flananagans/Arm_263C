#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "KeyBot.h"
#include "controller.h"

namespace Traj{

  void start(void);
  bool getCoord(float* pos, char letter);
  void setKey(char letter);
  void generateTrajectory(char letter);
  extern volatile bool traj_finished;
  
  const float keyCoords[28][2] = { 7.1250,  4.0625,
                                   6.3750,  0.6875,
                                   6.3750,  2.1875,
                                   7.1250,  2.5625,
                                   7.8750,  2.7812,
                                   7.1250,  1.8125,
                                   7.1250,  1.0625,
                                   7.1250,  0.3125,
                                   7.8750, -0.9688,
                                   7.1250, -0.4375,
                                   7.1250, -1.1875,
                                   7.1250, -1.9375,
                                   6.3750, -0.8125,
                                   6.3750, -0.0625,
                                   7.8750, -1.7188,
                                   7.8750, -2.4688,
                                   7.8750,  4.2812,
                                   7.8750,  2.0312,
                                   7.1250,  3.3125,
                                   7.8750,  1.2812,
                                   7.8750,  0.2188,
                                   6.3750,  1.4375,
                                   7.8750,  3.5312,
                                   6.3750,  2.9375,
                                   7.8750,  0.5312,
                                   6.3750,  3.6875,
                                   5.5000,  0.5938,
                                        0,       0}; // (x, y) coords for each character
      }

#endif
