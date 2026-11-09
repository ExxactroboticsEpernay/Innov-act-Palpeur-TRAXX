#include "board_io.h"

#if !PALPEUR_SIMULATION
#if !__has_include("board_19r_map.h")
#error "Add board_19r_map.h from the verified Industrial Shields 19R input map before enabling real acquisition."
#endif
#include "board_19r_map.h"
#endif

namespace palpeur {
namespace firmware {

void beginBoardInputs() {
#if !PALPEUR_SIMULATION
  palpeur19rBegin();
#endif
}

float readLeftSensorVoltage() {
#if PALPEUR_SIMULATION
  return 2.5f;
#else
  return palpeur19rReadLeftVoltage();
#endif
}

float readRightSensorVoltage() {
#if PALPEUR_SIMULATION
  return 2.5f;
#else
  return palpeur19rReadRightVoltage();
#endif
}

}  // namespace firmware
}  // namespace palpeur