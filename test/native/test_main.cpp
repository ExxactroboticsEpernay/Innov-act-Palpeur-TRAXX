#include "core/calibration.h"
#include "core/can_bus.h"
#include "core/can_protocol.h"
#include "core/filters.h"
#include "core/geometry.h"
#include "core/sensor_pipeline.h"
#include "sim/geometry_simulation.h"

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void check(bool condition, const char* name) {
  if (condition) {
    std::cout << "[OK] " << name << '\n';
  } else {
    std::cerr << "[FAIL] " << name << '\n';
    ++failures;
  }
}

bool near(float actual, float expected, float tolerance = 0.001f) {
  return std::fabs(actual - expected) <= tolerance;
}

palpeur::PairInput validInput(float left_angle, float right_angle) {
  palpeur::PairInput input;
  input.left_angle_deg = left_angle;
  input.right_angle_deg = right_angle;
  input.left_contact = true;
  input.right_contact = true;
  return input;
}

void testCalibration() {
  palpeur::SensorCalibration calibration;
  calibration.voltage_min_v = 0.5f;
  calibration.voltage_neutral_v = 2.5f;
  calibration.voltage_max_v = 4.5f;
  calibration.calibrated = true;

  const auto low = palpeur::convertVoltageToAngle(0.5f, calibration);
  const auto neutral = palpeur::convertVoltageToAngle(2.5f, calibration);
  const auto high = palpeur::convertVoltageToAngle(4.5f, calibration);
  check(low.valid && near(low.angle_deg, -90.0f), "voltage minimum maps to -90 degrees");
  check(neutral.valid && near(neutral.angle_deg, 0.0f), "voltage neutral maps to zero");
  check(high.valid && near(high.angle_deg, 90.0f), "voltage maximum maps to +90 degrees");
  check(palpeur::convertVoltageToAngle(0.2f, calibration).fault ==
            palpeur::SensorFault::VoltageOutOfRange,
        "voltage below fault window is rejected");

  calibration.invert = true;
  check(near(palpeur::convertVoltageToAngle(0.5f, calibration).angle_deg,
             90.0f),
        "inverted channel reverses angle sign");

  const float stable_samples[] = {2.49f, 2.50f, 2.51f};
  float captured_mean = 0.0f;
  check(palpeur::captureStableVoltage(stable_samples, 3, 0.03f, captured_mean) &&
            near(captured_mean, 2.50f),
        "stable neutral capture returns mean voltage");
  const float unstable_samples[] = {2.2f, 2.5f, 2.8f};
  check(!palpeur::captureStableVoltage(unstable_samples, 3, 0.05f, captured_mean),
        "unstable neutral capture is rejected");
}

void testContactHysteresis() {
  palpeur::ContactDetector detector(3.0f, 1.0f);
  palpeur::AngleReading reading;
  reading.valid = true;
  reading.neutral_angle_deg = 12.0f;
  reading.angle_deg = 14.0f;
  check(!detector.update(reading), "contact stays free below threshold");
  reading.angle_deg = 15.1f;
  check(detector.update(reading), "contact enters above threshold");
  reading.angle_deg = 14.5f;
  check(detector.update(reading), "hysteresis keeps contact near threshold");
  reading.angle_deg = 13.9f;
  check(!detector.update(reading), "contact releases below hysteresis threshold");
  reading.angle_deg = 20.0f;
  detector.update(reading);
  reading.valid = false;
  check(!detector.update(reading), "sensor invalidity clears contact state");
}

void testFilters() {
  palpeur::MedianFilter median(3);
  median.update(1.0f);
  median.update(2.0f);
  check(near(median.update(100.0f), 2.0f), "median filter rejects isolated high spike");

  palpeur::ExponentialFilter exponential(0.5f);
  exponential.update(10.0f);
  check(near(exponential.update(20.0f), 15.0f), "exponential filter smooths step");

  palpeur::SlewRateLimiter limiter(2.0f);
  limiter.update(0.0f, 0.1f);
  check(near(limiter.update(10.0f, 0.5f), 1.0f), "slew limiter bounds rate of change");
}

  void testSensorPipeline() {
    palpeur::SensorChannelConfig config;
    config.calibration.voltage_min_v = 0.5f;
    config.calibration.voltage_neutral_v = 2.5f;
    config.calibration.voltage_max_v = 4.5f;
    config.calibration.calibrated = true;
    config.smoothing_alpha = 1.0f;
    config.maximum_slew_deg_per_second = 1000.0f;
    palpeur::SensorChannel channel(config);

    const auto neutral = channel.update(2.5f, 0.1f);
    check(neutral.valid && !neutral.contact && near(neutral.filtered_angle_deg, 0.0f),
      "sensor pipeline reports calibrated neutral");
    const auto contact = channel.update(3.0f, 0.1f);
    check(contact.valid && contact.contact,
      "sensor pipeline detects contact from calibrated angle");
    const auto fault = channel.update(0.1f, 0.1f);
    check(!fault.valid && !fault.contact &&
      fault.fault == palpeur::SensorFault::VoltageOutOfRange,
      "sensor pipeline clears contact after voltage fault");
  }

void testEstimatorsAndValidity() {
  palpeur::PairConfig config;
  config.pivot_spacing_m = 0.525f;
  config.contact_length_m = 0.35f;
  palpeur::PairInput input = validInput(30.0f, 30.0f);

  palpeur::ContactPointEstimator contact_estimator;
  const auto contact = contact_estimator.estimate(config, input);
  check(contact.valid && near(contact.row_y_m, 0.0f),
        "v1 symmetric pair estimates centered row");
  check(near(contact.apparent_width_m, 0.175f),
        "v1 reports apparent contact width");

  palpeur::BisectorEstimator bisector_estimator;
  const auto bisector = bisector_estimator.estimate(config, input);
  check(bisector.valid && near(bisector.row_y_m, 0.0f),
        "v2 bisector intersects pair station");

  input.right_contact = false;
  const auto one_arm = contact_estimator.estimate(config, input);
  check(!one_arm.valid && one_arm.reason == palpeur::InvalidReason::RightFree,
        "pair is invalid when right arm is free");

  input = validInput(30.0f, 30.0f);
  input.left_sensor_valid = false;
  const auto sensor_fault = contact_estimator.estimate(config, input);
  check(!sensor_fault.valid &&
            sensor_fault.reason == palpeur::InvalidReason::LeftSensorFault,
        "sensor fault is distinct from lost contact");
}

void testPose() {
  palpeur::VehicleGeometry geometry;
  geometry.wheelbase_m = 3.85f;
  geometry.rear_pair_offset_m = -0.40f;
  geometry.front_pair_offset_m = 0.40f;
  palpeur::PairEstimate front;
  front.valid = true;
  front.x_m = 4.25f;
  front.row_y_m = 0.12f;
  palpeur::PairEstimate rear;
  rear.reason = palpeur::InvalidReason::BothFree;

  auto pose = palpeur::estimateRowPose(geometry, rear, front);
  check(pose.lateral_valid && !pose.heading_valid &&
            near(pose.offset_at_valid_pair_m, -0.12f),
        "one valid pair supplies local offset only");

  rear.valid = true;
  rear.x_m = -0.40f;
  rear.row_y_m = 0.02f;
  pose = palpeur::estimateRowPose(geometry, rear, front);
  check(pose.lateral_valid && pose.heading_valid && pose.heading_deg < 0.0f,
        "two-pair pose reports heading with expected sign");
  check(near(pose.offset_rear_axle_m, -0.028108f, 0.002f),
        "two-pair interpolation computes rear axle offset");
}

  void testCanProtocol() {
    palpeur::CanPoseState source;
    source.lateral_offset_m = -0.1234f;
    source.heading_deg = -1.23f;
    source.lateral_valid = true;
    source.heading_valid = false;
    source.front_pair_valid = true;
    source.calibration_valid = true;
    source.simulation_mode = true;
    source.counter = 254;
    const auto encoded_pose = palpeur::encodePose(source);
    palpeur::CanPoseState decoded_pose;
    check(palpeur::decodePose(encoded_pose, decoded_pose),
      "CAN pose frame decodes successfully");
    check(near(decoded_pose.lateral_offset_m, -0.1234f, 0.0001f) &&
      decoded_pose.front_pair_valid && !decoded_pose.heading_valid &&
      decoded_pose.counter == 254,
      "CAN pose round-trip preserves scale, validity and counter");

    palpeur::CanFrame wrong_frame = encoded_pose;
    wrong_frame.dlc = 7;
    check(!palpeur::decodePose(wrong_frame, decoded_pose),
      "CAN pose decoder rejects wrong payload length");

    palpeur::CanArmState arms;
    arms.left_angle_deg = -24.12f;
    arms.right_angle_deg = 24.56f;
    arms.left_contact = true;
    arms.pair_valid = true;
    arms.invalid_reason = palpeur::CanInvalidReason::RightFree;
    arms.counter = 17;
    const auto arm_frame = palpeur::encodeArms(palpeur::CanId::FrontArms, arms);
    check(arm_frame.id == static_cast<std::uint16_t>(palpeur::CanId::FrontArms) &&
              arm_frame.data[0] == 0x94u && arm_frame.data[1] == 0xF6u &&
      arm_frame.data[6] == 17,
      "CAN arm frame uses little-endian centidegrees and counter");
    palpeur::CanArmState decoded_arms;
    check(palpeur::decodeArms(arm_frame, palpeur::CanId::FrontArms, decoded_arms) &&
              near(decoded_arms.left_angle_deg, -24.12f) &&
              decoded_arms.left_contact && decoded_arms.pair_valid &&
              decoded_arms.invalid_reason == palpeur::CanInvalidReason::RightFree,
          "CAN arm frame round-trip preserves values and flags");

    palpeur::CanStatusState status;
    status.simulation_mode = true;
    status.front_pair_enabled = true;
    status.estimator = 2;
    status.quality = 1;
    status.calibration_valid = true;
    const auto status_frame = palpeur::encodeStatus(status);
    check(status_frame.data[0] == 0xB3u,
      "CAN status packs mode, pair, estimator, quality and calibration");
      palpeur::CanStatusState decoded_status;
      check(palpeur::decodeStatus(status_frame, decoded_status) &&
        decoded_status.simulation_mode && decoded_status.front_pair_enabled &&
        !decoded_status.rear_pair_enabled && decoded_status.estimator == 2 &&
        decoded_status.quality == 1 && decoded_status.calibration_valid,
        "CAN status frame round-trip preserves bit fields");

    const auto heartbeat = palpeur::encodeHeartbeat(9, 1, 2, 3);
    check(heartbeat.id == static_cast<std::uint16_t>(palpeur::CanId::Heartbeat) &&
      heartbeat.data[0] == 9 && heartbeat.data[3] == 3,
      "CAN heartbeat carries counter and semantic version");
      palpeur::CanFirmwareVersion version;
      check(palpeur::decodeHeartbeat(heartbeat, version) && version.counter == 9 &&
        version.major == 1 && version.minor == 2 && version.patch == 3,
        "CAN heartbeat frame round-trip preserves firmware version");

    palpeur::CanOffsets offsets;
    offsets.rear_axle_m = -0.2f;
    offsets.wheelbase_center_m = 0.0f;
    offsets.front_axle_m = 0.2f;
    offsets.counter = 4;
    const auto offsets_frame = palpeur::encodeOffsets(offsets);
    check(offsets_frame.data[0] == 0x30u && offsets_frame.data[1] == 0xF8u &&
      offsets_frame.data[4] == 0xD0u && offsets_frame.data[5] == 0x07u,
      "CAN offsets use signed 0.1 mm scaling");
    palpeur::CanOffsets decoded_offsets;
    check(palpeur::decodeOffsets(offsets_frame, decoded_offsets) &&
              near(decoded_offsets.rear_axle_m, -0.2f) &&
              near(decoded_offsets.front_axle_m, 0.2f) &&
              decoded_offsets.counter == 4,
          "CAN offsets frame round-trip preserves all references");
  }

void testGeometrySimulation() {
  palpeur::CircularObjectScenario scenario;
  scenario.pair.pivot_spacing_m = 0.525f;
  scenario.pair.contact_length_m = 0.35f;
  scenario.diameter_m = 0.10f;
  scenario.row_y_at_pair_m = 0.08f;
  scenario.row_slope = 0.02f;
  const auto contact = palpeur::simulateCircularObjectContact(scenario);
  check(contact.valid && contact.input.left_contact && contact.input.right_contact,
        "circle simulator generates two tangent arm contacts");
  if (contact.valid) {
    palpeur::ContactPointEstimator estimator;
    const auto estimate = estimator.estimate(scenario.pair, contact.input);
    check(estimate.valid && std::isfinite(estimate.row_y_m),
          "simulated contact is accepted by the v1 estimator");
  }
}

// Bout en bout à deux paires : rang droit, machine décalée et en lacet connu.
// Le simulateur génère les angles des 4 bras, l'estimateur v1 puis
// estimateRowPose doivent retrouver le cap et le décalage.
void testTwoPairSimulation() {
  constexpr float kPi = 3.14159265358979323846f;
  palpeur::VehicleGeometry geometry;
  geometry.wheelbase_m = 2.14f;
  geometry.rear_pair_offset_m = -0.40f;
  geometry.front_pair_offset_m = 0.40f;

  const float heading_deg = 1.5f;              // ψ, cap machine / rang
  const float row_slope = std::tan(-heading_deg * kPi / 180.0f);  // α = -ψ
  // Rang 5 cm à gauche de O : environ +6 cm à la paire AR, -2 cm à la paire AV.
  const float row_y_at_rear_axle_m = 0.05f;

  palpeur::PairEstimate estimates[2];
  const float pair_x[2] = {geometry.rear_pair_offset_m,
                           geometry.wheelbase_m + geometry.front_pair_offset_m};
  palpeur::ContactPointEstimator estimator;
  bool simulated_ok = true;
  for (int index = 0; index < 2; ++index) {
    palpeur::CircularObjectScenario scenario;
    scenario.pair.x_m = pair_x[index];
    scenario.diameter_m = 0.05f;
    scenario.row_y_at_pair_m = row_y_at_rear_axle_m + row_slope * pair_x[index];
    scenario.row_slope = row_slope;
    const auto contact = palpeur::simulateCircularObjectContact(scenario);
    simulated_ok = simulated_ok && contact.valid;
    estimates[index] = estimator.estimate(scenario.pair, contact.input);
  }
  check(simulated_ok && estimates[0].valid && estimates[1].valid,
        "two simulated pairs produce two valid estimates");

  const auto pose = palpeur::estimateRowPose(geometry, estimates[0], estimates[1]);
  check(pose.heading_valid && near(pose.heading_deg, heading_deg, 0.3f),
        "two-pair simulation recovers heading within 0.3 deg");
  check(pose.lateral_valid &&
            near(pose.offset_rear_axle_m, -row_y_at_rear_axle_m, 0.02f),
        "two-pair simulation recovers rear axle offset within 2 cm");
}

void testDisabledPairReason() {
  palpeur::VehicleGeometry geometry;
  palpeur::PairEstimate rear;
  rear.reason = palpeur::InvalidReason::PairDisabled;
  palpeur::PairEstimate front;
  front.reason = palpeur::InvalidReason::LeftFree;
  auto pose = palpeur::estimateRowPose(geometry, rear, front);
  check(!pose.lateral_valid && pose.reason == palpeur::InvalidReason::LeftFree,
        "disabled rear pair does not hide the front pair reason");

  front.reason = palpeur::InvalidReason::PairDisabled;
  rear.reason = palpeur::InvalidReason::RightFree;
  pose = palpeur::estimateRowPose(geometry, rear, front);
  check(!pose.lateral_valid && pose.reason == palpeur::InvalidReason::RightFree,
        "disabled front pair does not hide the rear pair reason");
}

void testDisabledCanBus() {
  palpeur::DisabledCanBus bus;
  palpeur::CanFrame frame;
  check(!bus.begin(500000) && !bus.enabled(),
        "CAN stub remains disabled by default");
  check(!bus.send(frame) && !bus.receive(frame),
        "CAN stub cannot transmit or receive frames");
}

}  // namespace

int main() {
  testCalibration();
  testContactHysteresis();
  testFilters();
  testSensorPipeline();
  testEstimatorsAndValidity();
  testPose();
  testCanProtocol();
  testGeometrySimulation();
  testTwoPairSimulation();
  testDisabledPairReason();
  testDisabledCanBus();
  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return 1;
  }
  std::cout << "All native tests passed\n";
  return 0;
}