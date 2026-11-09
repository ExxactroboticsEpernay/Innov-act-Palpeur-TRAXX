#pragma once

#ifndef PALPEUR_SIMULATION
#define PALPEUR_SIMULATION 1
#endif

namespace palpeur {
namespace firmware {

// Voies capteurs, dans l'ordre utilisé partout (firmware, JSON, NVS) :
// 0 = avant gauche, 1 = avant droit, 2 = arrière gauche, 3 = arrière droit.
constexpr int kSensorChannelCount = 4;

void beginBoardInputs();
// Tension en volts côté entrée analogique, ou NAN si la voie n'est pas câblée.
float readSensorVoltage(int channel);

}  // namespace firmware
}  // namespace palpeur
