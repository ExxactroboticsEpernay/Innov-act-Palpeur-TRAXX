#pragma once

#include "calibration.h"
#include "filters.h"

namespace palpeur {

struct SensorChannelConfig {
  SensorCalibration calibration;
  float contact_threshold_deg = 3.0f;
  float contact_hysteresis_deg = 1.0f;
  float smoothing_alpha = 0.25f;
  float maximum_slew_deg_per_second = 60.0f;
  std::size_t median_window = 5;
};

struct SensorChannelReading {
  bool valid = false;
  bool contact = false;
  float voltage_v = 0.0f;
  float raw_angle_deg = 0.0f;
  float filtered_angle_deg = 0.0f;
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
        contact_(config.contact_threshold_deg, config.contact_hysteresis_deg) {}

  SensorChannelReading update(float voltage_v, float delta_seconds) {
    SensorChannelReading output;
    output.voltage_v = voltage_v;
    const AngleReading raw = convertVoltageToAngle(voltage_v, config_.calibration);
    output.fault = raw.fault;
    if (!raw.valid) {
      median_.reset();
      smoothing_.reset();
      slew_.reset();
      contact_.reset();
      return output;
    }

    const float median_angle = median_.update(raw.angle_deg);
    const float smooth_angle = smoothing_.update(median_angle);
    output.raw_angle_deg = raw.angle_deg;
    output.filtered_angle_deg = slew_.update(smooth_angle, delta_seconds);
    output.neutral_angle_deg = raw.neutral_angle_deg;
    AngleReading filtered = raw;
    filtered.angle_deg = output.filtered_angle_deg;
    output.contact = contact_.update(filtered);
    output.valid = true;
    return output;
  }

  void reset() {
    median_.reset();
    smoothing_.reset();
    slew_.reset();
    contact_.reset();
  }

 private:
  SensorChannelConfig config_;
  MedianFilter median_;
  ExponentialFilter smoothing_;
  SlewRateLimiter slew_;
  ContactDetector contact_;
};

}  // namespace palpeur