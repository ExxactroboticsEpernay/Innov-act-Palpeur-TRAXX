#pragma once

#include "core/measurement.h"

namespace palpeur {

struct CircularObjectScenario {
  PairConfig pair;
  float diameter_m = 0.10f;
  float row_y_at_pair_m = 0.0f;
  float row_slope = 0.0f;
};

struct SimulatedContact {
  bool valid = false;
  float object_center_x_m = 0.0f;
  float object_center_y_m = 0.0f;
  PairInput input;
};

SimulatedContact simulateCircularObjectContact(
    const CircularObjectScenario& scenario);

}  // namespace palpeur