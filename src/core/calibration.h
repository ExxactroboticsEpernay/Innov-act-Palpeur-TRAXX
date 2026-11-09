#pragma once

#include <cstddef>

namespace palpeur {

enum class SensorFault {
  None,
  NonFiniteVoltage,
  VoltageOutOfRange,
  CalibrationInvalid
};

// Gain nominal du capteur Honeywell RTY180HVNAA : 0,5 V → 4,5 V pour 180°.
constexpr float kNominalVoltsPerDegree = 4.0f / 180.0f;

// `invert` : choisi à la calibration pour que l'angle soit positif quand le
// bras est repoussé vers l'arrière (machine en marche avant).
struct SensorCalibration {
  float voltage_min_v = 0.5f;
  float voltage_neutral_v = 2.5f;
  float voltage_max_v = 4.5f;
  bool invert = false;
  bool calibrated = false;
};

struct AngleReading {
  bool valid = false;
  SensorFault fault = SensorFault::None;
  float angle_deg = 0.0f;
  float neutral_angle_deg = 0.0f;
};

bool isCalibrationValid(const SensorCalibration& calibration);
AngleReading convertVoltageToAngle(float voltage_v,
                                   const SensorCalibration& calibration);
bool captureStableVoltage(const float* samples, std::size_t sample_count,
                          float maximum_spread_v, float& mean_voltage_v);

class ContactDetector {
 public:
  ContactDetector(float threshold_deg = 3.0f, float hysteresis_deg = 1.0f);
  bool update(const AngleReading& reading);
  void reset();
  bool inContact() const;

 private:
  float threshold_deg_;
  float hysteresis_deg_;
  bool in_contact_ = false;
};

}  // namespace palpeur