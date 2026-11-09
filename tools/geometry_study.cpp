#include "core/geometry.h"
#include "sim/geometry_simulation.h"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
  palpeur::ContactPointEstimator v1;
  palpeur::BisectorEstimator v2;
  palpeur::PairConfig pair;
  pair.pivot_spacing_m = 0.525f;
  pair.contact_length_m = 0.35f;

  std::cout << "diameter_cm,offset_cm,yaw_deg,v1_error_cm,v2_error_cm\n";
  std::cout << std::fixed << std::setprecision(3);
  for (int diameter_cm = 1; diameter_cm <= 30; ++diameter_cm) {
    for (int offset_cm = -20; offset_cm <= 20; offset_cm += 10) {
      for (int yaw_deg = -5; yaw_deg <= 5; yaw_deg += 5) {
        palpeur::CircularObjectScenario scenario;
        scenario.pair = pair;
        scenario.diameter_m = diameter_cm / 100.0f;
        scenario.row_y_at_pair_m = offset_cm / 100.0f;
        scenario.row_slope = std::tan(yaw_deg * 3.14159265358979323846f / 180.0f);
        const auto contact = palpeur::simulateCircularObjectContact(scenario);
        if (!contact.valid) continue;

        const auto estimate_v1 = v1.estimate(pair, contact.input);
        const auto estimate_v2 = v2.estimate(pair, contact.input);
        if (!estimate_v1.valid || !estimate_v2.valid) continue;

        const float true_y_at_station = scenario.row_y_at_pair_m;
        std::cout << diameter_cm << ',' << offset_cm << ',' << yaw_deg << ','
                  << (estimate_v1.row_y_m - true_y_at_station) * 100.0f << ','
                  << (estimate_v2.row_y_m - true_y_at_station) * 100.0f << '\n';
      }
    }
  }
  return 0;
}