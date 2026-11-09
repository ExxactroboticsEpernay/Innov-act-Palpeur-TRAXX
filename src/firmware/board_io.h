#pragma once

#ifndef PALPEUR_SIMULATION
#define PALPEUR_SIMULATION 1
#endif

namespace palpeur {
namespace firmware {

void beginBoardInputs();
float readLeftSensorVoltage();
float readRightSensorVoltage();

}  // namespace firmware
}  // namespace palpeur