#pragma once

#include "calibration.h"
#include "filters.h"

#include <algorithm>
#include <cmath>

namespace palpeur {

struct SensorChannelConfig {
  SensorCalibration calibration;
  float contact_threshold_deg = 3.0f;
  float contact_hysteresis_deg = 1.0f;
  float smoothing_alpha = 0.25f;
  // Un cep à ~1 m/s balaie le bras d'environ 50° en 0,3 s : la limite de pente
  // doit rester au-dessus (~160°/s) pour ne pas retarder la mesure.
  float maximum_slew_deg_per_second = 300.0f;
  std::size_t median_window = 5;
  // Anti-claquement. Quand le cep libère le bras, les ressorts le ramènent
  // brutalement : dépassement du repos et rebonds de part et d'autre.
  // - un contact n'est retenu que s'il dure au moins contact_confirm_s ;
  // - après une libération, aucun nouveau contact pendant release_blanking_s.
  float contact_confirm_s = 0.04f;
  float release_blanking_s = 0.25f;
};

struct SensorChannelReading {
  bool valid = false;
  bool contact = false;
  bool rebound = false;  // fenêtre anti-rebond active après une libération
  float voltage_v = 0.0f;
  float raw_angle_deg = 0.0f;       // balayage brut depuis le neutre
  float filtered_angle_deg = 0.0f;  // balayage filtré
  float neutral_angle_deg = 0.0f;
  SensorFault fault = SensorFault::None;
};

class SensorChannel {
 public:
  explicit SensorChannel(const SensorChannelConfig& config)
      : config_(config),
        median_(config.median_window),
        smoothing_(config.smoothing_alpha),
        slew_(config.maximum_slew_deg_per_second),
        threshold_(config.contact_threshold_deg, config.contact_hysteresis_deg) {}

  SensorChannelReading update(float voltage_v, float delta_seconds) {
    SensorChannelReading output;
    output.voltage_v = voltage_v;
    const AngleReading raw = convertVoltageToAngle(voltage_v, config_.calibration);
    output.fault = raw.fault;
    if (!raw.valid) {
      reset();
      return output;
    }

    const float step = std::isfinite(delta_seconds) && delta_seconds > 0.0f
        ? delta_seconds
        : 0.0f;
    const float median_angle = median_.update(raw.angle_deg);

    // Le contact est décidé sur la médiane (robuste aux pics), pas sur la
    // valeur lissée : un retour brutal du bras doit couper le contact tout de suite.
    AngleReading median_reading = raw;
    median_reading.angle_deg = median_angle;
    const bool beyond_threshold = threshold_.update(median_reading);

    // Côté du balayage : + vers l'arrière (marche avant), - vers l'avant
    // (marche arrière). Un passage de l'autre côté du repos = libération.
    const int side = median_angle >= 0.0f ? 1 : -1;
    blanking_left_s_ = std::max(0.0f, blanking_left_s_ - step);
    if (in_contact_) {
      if (!beyond_threshold || side != contact_side_) {
        // Libération : le ressort ramène le bras. On ignore les rebonds qui
        // suivent et on repart du point courant, sans traîner l'ancien angle.
        in_contact_ = false;
        confirm_s_ = 0.0f;
        blanking_left_s_ = config_.release_blanking_s;
        smoothing_.reset();
        slew_.reset();
      }
    } else if (beyond_threshold && blanking_left_s_ <= 0.0f) {
      if (confirm_s_ > 0.0f && side != contact_side_) confirm_s_ = 0.0f;  // changement de côté
      contact_side_ = side;
      confirm_s_ += step;
      if (confirm_s_ + 1e-6f >= config_.contact_confirm_s) in_contact_ = true;
    } else {
      confirm_s_ = 0.0f;
    }

    const float smooth_angle = smoothing_.update(median_angle);
    output.raw_angle_deg = raw.angle_deg;
    output.filtered_angle_deg = slew_.update(smooth_angle, delta_seconds);
    output.neutral_angle_deg = raw.neutral_angle_deg;
    output.contact = in_contact_;
    output.rebound = blanking_left_s_ > 0.0f;
    output.valid = true;
    return output;
  }

  void reset() {
    median_.reset();
    smoothing_.reset();
    slew_.reset();
    threshold_.reset();
    in_contact_ = false;
    confirm_s_ = 0.0f;
    blanking_left_s_ = 0.0f;
  }

 private:
  SensorChannelConfig config_;
  MedianFilter median_;
  ExponentialFilter smoothing_;
  SlewRateLimiter slew_;
  ContactDetector threshold_;  // seuil + hystérésis sur |balayage|
  bool in_contact_ = false;
  int contact_side_ = 1;
  float confirm_s_ = 0.0f;
  float blanking_left_s_ = 0.0f;
};

}  // namespace palpeur
