#include "board_io.h"

#include <cmath>

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

float readSensorVoltage(int channel) {
  if (channel < 0 || channel >= kSensorChannelCount) return NAN;
#if PALPEUR_SIMULATION
  return NAN;  // en simulation, les tensions sont générées par application.cpp
#else
  // Doit renvoyer NAN pour une voie non câblée : elle apparaît alors en
  // défaut capteur au lieu de produire une fausse mesure.
  return palpeur19rReadVoltage(channel);
#endif
}

}  // namespace firmware
}  // namespace palpeur
