#include "trajectory.h"

namespace Traj {

  bool getCoord(float* pos, char letter) {
    if(letter == '1') { // Space bar is mapped to the character '1'
      memcpy(&pos[0], &keyCoords[26][0], 2*sizeof(float));
      return true;
    } else if( letter > 64 && letter < 91) { // Use ASCII code to index
      memcpy(&pos[0], &keyCoords[(uint8_t)letter - 65][0], 2*sizeof(float));
    }
    return false;
  }
}
