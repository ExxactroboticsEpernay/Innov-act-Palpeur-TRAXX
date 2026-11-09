#pragma once

#include <cmath>
#include <cstdint>

namespace palpeur {

// Mesure par cep.
//
// Rang type : un cep tous les 1 m, un piquet tous les 5 à 10 m (entre deux
// ceps), TRAXX jusqu'à 6 km/h (1,67 m/s). Une paire n'est en double contact
// que pendant le passage d'un objet : ~0,3 m de trajet, ~180 ms à 6 km/h.
// Entre deux objets, les bras sont libres. On résume donc chaque passage en
// une mesure, conservée jusqu'au suivant, au lieu de publier un flux qui
// alterne valide / invalide.
struct VineMeasurement {
  bool available = false;      // au moins un passage complet mesuré
  float lr_diff_deg = 0.0f;    // moyenne de φG − φD pendant le passage
  float row_y_m = 0.0f;        // moyenne de y_row pendant le passage
  float peak_sweep_deg = 0.0f; // balayage moyen G/D maximal pendant le passage
  float duration_s = 0.0f;     // durée du double contact
  float age_s = 0.0f;          // temps écoulé depuis la fin du passage
  std::uint32_t count = 0;     // nombre de passages mesurés
};

class VineEventTracker {
 public:
  // min_duration_s : un double contact plus court est ignoré (choc, rebond).
  explicit VineEventTracker(float min_duration_s = 0.03f)
      : min_duration_s_(min_duration_s) {}

  // À appeler à chaque échantillon. `pair_valid` : double contact valide.
  // Retourne true quand un passage vient de se terminer et a été retenu.
  bool update(bool pair_valid, float lr_diff_deg, float row_y_m,
              float mean_sweep_deg, float delta_seconds) {
    const float step = std::isfinite(delta_seconds) && delta_seconds > 0.0f
        ? delta_seconds
        : 0.0f;
    if (last_.available) last_.age_s += step;

    const bool usable = pair_valid && std::isfinite(lr_diff_deg) &&
                        std::isfinite(row_y_m);
    if (usable) {
      if (!in_event_) {
        in_event_ = true;
        duration_s_ = 0.0f;
        sum_lr_diff_ = 0.0;
        sum_row_y_ = 0.0;
        samples_ = 0;
        peak_sweep_deg_ = 0.0f;
      }
      duration_s_ += step;
      sum_lr_diff_ += lr_diff_deg;
      sum_row_y_ += row_y_m;
      ++samples_;
      if (std::isfinite(mean_sweep_deg) && std::fabs(mean_sweep_deg) > std::fabs(peak_sweep_deg_)) {
        peak_sweep_deg_ = mean_sweep_deg;
      }
      return false;
    }

    if (!in_event_) return false;
    in_event_ = false;
    if (samples_ == 0 || duration_s_ + 1e-6f < min_duration_s_) return false;
    last_.available = true;
    last_.lr_diff_deg = static_cast<float>(sum_lr_diff_ / samples_);
    last_.row_y_m = static_cast<float>(sum_row_y_ / samples_);
    last_.peak_sweep_deg = peak_sweep_deg_;
    last_.duration_s = duration_s_;
    last_.age_s = 0.0f;
    ++last_.count;
    return true;
  }

  const VineMeasurement& last() const { return last_; }
  bool inEvent() const { return in_event_; }
  // Mesure récente : moins de max_age_s depuis la fin du dernier passage.
  bool fresh(float max_age_s) const {
    return last_.available && last_.age_s <= max_age_s;
  }

  void reset() {
    last_ = VineMeasurement{};
    in_event_ = false;
    samples_ = 0;
  }

 private:
  float min_duration_s_;
  VineMeasurement last_;
  bool in_event_ = false;
  float duration_s_ = 0.0f;
  double sum_lr_diff_ = 0.0;
  double sum_row_y_ = 0.0;
  std::uint32_t samples_ = 0;
  float peak_sweep_deg_ = 0.0f;
};

}  // namespace palpeur
