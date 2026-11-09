#pragma once

#include "measurement.h"

#include <cstdint>

namespace palpeur {

enum class CanId : std::uint16_t {
  Pose = 0x3A0,
  FrontArms = 0x3A1,
  RearArms = 0x3A2,
  Status = 0x3A3,
  Heartbeat = 0x3A4,
  Offsets = 0x3A5
};

enum class CanInvalidReason : std::uint8_t {
  None = 0,
  LeftFree = 1,
  RightFree = 2,
  BothFree = 3,
  SensorFault = 4,
  OutOfRange = 5,
  CalibrationInvalid = 6,
  PairDisabled = 7
};

struct CanFrame {
  std::uint16_t id = 0;
  std::uint8_t dlc = 8;
  std::uint8_t data[8]{};
};

struct CanPoseState {
  float lateral_offset_m = 0.0f;
  float heading_deg = 0.0f;
  bool lateral_valid = false;
  bool heading_valid = false;
  bool front_pair_valid = false;
  bool rear_pair_valid = false;
  bool estimated = false;
  bool sensor_fault = false;
  bool calibration_valid = false;
  bool simulation_mode = false;
  std::uint8_t counter = 0;
};

struct CanArmState {
  float left_angle_deg = 0.0f;
  float right_angle_deg = 0.0f;
  bool left_contact = false;
  bool right_contact = false;
  bool pair_valid = false;
  bool left_fault = false;
  bool right_fault = false;
  bool estimated = false;
  CanInvalidReason invalid_reason = CanInvalidReason::None;
  std::uint8_t counter = 0;
};

struct CanStatusState {
  bool simulation_mode = false;
  bool front_pair_enabled = true;
  bool rear_pair_enabled = false;
  std::uint8_t estimator = 1;
  std::uint8_t quality = 0;
  bool calibration_valid = false;
};

struct CanFirmwareVersion {
  std::uint8_t counter = 0;
  std::uint8_t major = 0;
  std::uint8_t minor = 0;
  std::uint8_t patch = 0;
};

struct CanOffsets {
  float rear_axle_m = 0.0f;
  float wheelbase_center_m = 0.0f;
  float front_axle_m = 0.0f;
  std::uint8_t counter = 0;
};

CanFrame encodePose(const CanPoseState& state);
bool decodePose(const CanFrame& frame, CanPoseState& state);
CanFrame encodeArms(CanId id, const CanArmState& state);
bool decodeArms(const CanFrame& frame, CanId id, CanArmState& state);
CanFrame encodeStatus(const CanStatusState& state);
bool decodeStatus(const CanFrame& frame, CanStatusState& state);
CanFrame encodeHeartbeat(std::uint8_t counter, std::uint8_t major,
                         std::uint8_t minor, std::uint8_t patch);
bool decodeHeartbeat(const CanFrame& frame, CanFirmwareVersion& version);
CanFrame encodeOffsets(const CanOffsets& offsets);
bool decodeOffsets(const CanFrame& frame, CanOffsets& offsets);

}  // namespace palpeur