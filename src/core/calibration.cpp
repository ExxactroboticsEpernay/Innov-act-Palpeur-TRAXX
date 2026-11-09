#include "calibration.h"

#include <algorithm>
#include <cmath>

namespace palpeur {
namespace {

constexpr float kMinimumSensorVoltage = 0.3f;
constexpr float kMaximumSensorVoltage = 4.7f;
// Les butées servent au contrôle de cohérence, pas au gain : la course du
// bras est limitée mécaniquement et ne couvre pas forcément les 180° du capteur.
constexpr float kMinimumCalibrationSpan = 1.0f;
constexpr float kMinimumStableSamples = 2.0f;

}  // namespace

bool isCalibrationValid(const SensorCalibration& calibration) {
  return calibration.calibrated &&
         std::isfinite(calibration.voltage_min_v) &&
         std::isfinite(calibration.voltage_neutral_v) &&
         std::isfinite(calibration.voltage_max_v) &&
         calibration.voltage_min_v >= kMinimumSensorVoltage &&
         calibration.voltage_max_v <= kMaximumSensorVoltage &&
         calibration.voltage_min_v < calibration.voltage_neutral_v &&
         calibration.voltage_neutral_v < calibration.voltage_max_v &&
         calibration.voltage_max_v - calibration.voltage_min_v >=
             kMinimumCalibrationSpan;
}

AngleReading convertVoltageToAngle(
    float voltage_v, const SensorCalibration& calibration) {
  AngleReading result;
  if (!std::isfinite(voltage_v)) {
    result.fault = SensorFault::NonFiniteVoltage;
    return result;
  }
  if (voltage_v < kMinimumSensorVoltage || voltage_v > kMaximumSensorVoltage) {
    result.fault = SensorFault::VoltageOutOfRange;
    return result;
  }
  if (!isCalibrationValid(calibration)) {
    result.fault = SensorFault::CalibrationInvalid;
    return result;
  }

  // RTY180HVNAA non ratiométrique : 4,0 V pour 180°, soit un gain fixe.
  // Angle relatif au neutre calibré (bras au repos), signe fixé par `invert`.
  const float direction = calibration.invert ? -1.0f : 1.0f;
  result.angle_deg = direction * (voltage_v - calibration.voltage_neutral_v) /
                     kNominalVoltsPerDegree;
  result.neutral_angle_deg = 0.0f;
  result.valid = true;
  return result;
}

bool captureStableVoltage(const float* samples, std::size_t sample_count,
                          float maximum_spread_v, float& mean_voltage_v) {
  if (samples == nullptr || sample_count < kMinimumStableSamples ||
      !std::isfinite(maximum_spread_v) || maximum_spread_v < 0.0f) {
    return false;
  }

  float minimum = samples[0];
  float maximum = samples[0];
  double sum = 0.0;
  for (std::size_t index = 0; index < sample_count; ++index) {
    const float sample = samples[index];
    if (!std::isfinite(sample) || sample < kMinimumSensorVoltage ||
        sample > kMaximumSensorVoltage) {
      return false;
    }
    minimum = std::min(minimum, sample);
    maximum = std::max(maximum, sample);
    sum += sample;
  }

  if (maximum - minimum > maximum_spread_v) return false;
  mean_voltage_v = static_cast<float>(sum / sample_count);
  return true;
}

ContactDetector::ContactDetector(float threshold_deg, float hysteresis_deg)
    : threshold_deg_(std::isfinite(threshold_deg) && threshold_deg > 0.0f
                         ? threshold_deg
                         : 3.0f),
      hysteresis_deg_(std::isfinite(hysteresis_deg) && hysteresis_deg >= 0.0f
                          ? std::min(hysteresis_deg, threshold_deg_)
                          : 1.0f) {}

bool ContactDetector::update(const AngleReading& reading) {
  if (!reading.valid || reading.fault != SensorFault::None ||
      !std::isfinite(reading.angle_deg) ||
      !std::isfinite(reading.neutral_angle_deg)) {
    in_contact_ = false;
    return in_contact_;
  }

  const float displacement = std::fabs(reading.angle_deg - reading.neutral_angle_deg);
  if (in_contact_) {
    if (displacement <= threshold_deg_ - hysteresis_deg_) in_contact_ = false;
  } else if (displacement >= threshold_deg_) {
    in_contact_ = true;
  }
  return in_contact_;
}

void ContactDetector::reset() { in_contact_ = false; }

bool ContactDetector::inContact() const { return in_contact_; }

}  // namespace palpeur