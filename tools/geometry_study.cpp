// Étude de sensibilité v1 / v2 sur le montage réel : b = 0,515 m, L = c = 0,395 m,
// bras perpendiculaires au repos et repoussés vers l'arrière par le cep.
// Sortie CSV : erreur d'estimation de y_row à la station, et écart G/D (φG - φD).
#include "../src/core/geometry.h"
#include "../src/sim/geometry_simulation.h"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kRestAngleDeg = 90.0f;
  palpeur::ContactPointEstimator v1;
  palpeur::BisectorEstimator v2;
  palpeur::PairConfig pair;  // convention firmware : θ depuis +x, s = +1
  pair.pivot_spacing_m = 0.515f;
  pair.contact_length_m = 0.395f;
  pair.arm_direction = 1;

  std::cout << "diameter_cm,offset_cm,yaw_deg,sweep_left_deg,sweep_right_deg,"
               "lr_diff_deg,v1_error_cm,v2_error_cm\n";
  std::cout << std::fixed << std::setprecision(3);
  for (int diameter_cm = 3; diameter_cm <= 25; ++diameter_cm) {  // ceps : 3 à 25 cm
    for (int offset_cm = -12; offset_cm <= 12; offset_cm += 3) {
      for (int yaw_deg = -5; yaw_deg <= 5; yaw_deg += 5) {
        palpeur::CircularObjectScenario scenario;
        scenario.pair = pair;
        scenario.pair.arm_direction = -1;  // simulateur : bras balayés vers l'arrière
        scenario.diameter_m = diameter_cm / 100.0f;
        scenario.row_y_at_pair_m = offset_cm / 100.0f;
        scenario.row_slope = std::tan(yaw_deg * kPi / 180.0f);
        const auto contact = palpeur::simulateCircularObjectContact(scenario);
        if (!contact.valid) continue;

        palpeur::PairInput input = contact.input;
        input.left_angle_deg = 180.0f - contact.input.left_angle_deg;
        input.right_angle_deg = 180.0f - contact.input.right_angle_deg;
        const auto estimate_v1 = v1.estimate(pair, input);
        const auto estimate_v2 = v2.estimate(pair, input);
        if (!estimate_v1.valid || !estimate_v2.valid) continue;

        const float sweep_left = input.left_angle_deg - kRestAngleDeg;
        const float sweep_right = input.right_angle_deg - kRestAngleDeg;
        const float true_y_at_station = scenario.row_y_at_pair_m;
        std::cout << diameter_cm << ',' << offset_cm << ',' << yaw_deg << ','
                  << sweep_left << ',' << sweep_right << ','
                  << sweep_left - sweep_right << ','
                  << (estimate_v1.row_y_m - true_y_at_station) * 100.0f << ','
                  << (estimate_v2.row_y_m - true_y_at_station) * 100.0f << '\n';
      }
    }
  }
  return 0;
}
