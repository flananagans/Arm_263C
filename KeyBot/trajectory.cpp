#include "trajectory.h"

namespace Traj {

  bool getCoord(float* pos, char letter) {
    if(letter == '1') { // Space bar is mapped to the character '1'
      memcpy(&pos[0], &keyCoords[26][0], 2*sizeof(float));
    } else if( letter > 64 && letter < 91) { // Use ASCII code to index
      memcpy(&pos[0], &keyCoords[(uint8_t)letter - 65][0], 2*sizeof(float));
    } else {
      return false;
    }
    // Convert units
    pos[0] = 0.0254*pos[0];
    pos[1] = 0.0254*pos[1];
    return true;
  }

  void setKey(char letter) {
    float p[2] = {0};
    if(getCoord(&p[0], letter)) {
      Controller::setPGoal(&p[0]);
    }
  }
  
}
