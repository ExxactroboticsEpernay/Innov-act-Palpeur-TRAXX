#include "can_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace palpeur {
namespace {

std::int16_t scaledSigned(float value, float factor) {
  if (!std::isfinite(value)) return 0;
  const float scaled = value * factor;
  const float bounded = std::max(
      static_cast<float>(std::numeric_limits<std::int16_t>::min()),
      std::min(scaled,
               static_cast<float>(std::numeric_limits<std::int16_t>::max())));
  return static_cast<std::int16_t>(std::lround(bounded));
}

void writeSigned(std::uint8_t* data, std::size_t offset, std::int16_t value) {
  const auto raw = static_cast<std::uint16_t>(value);
  data[offset] = static_cast<std::uint8_t>(raw & 0xFFu);
  data[offset + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFFu);
}

std::int16_t readSigned(const std::uint8_t* data, std::size_t offset) {
  const std::uint16_t raw = static_cast<std::uint16_t>(data[offset]) |
      (static_cast<std::uint16_t>(data[offset + 1]) << 8);
  return static_cast<std::int16_t>(raw);
}

CanFrame emptyFrame(CanId id) {
  CanFrame frame;
  frame.id = static_cast<std::uint16_t>(id);
  return frame;
}

}  // namespace

CanFrame encodePose(const CanPoseState& state) {
  CanFrame frame = emptyFrame(CanId::Pose);
  writeSigned(frame.data, 0,
              state.lateral_valid ? scaledSigned(state.lateral_offset_m, 10000.0f)
                                  : 0);
  writeSigned(frame.data, 2,
              state.heading_valid ? scaledSigned(state.heading_deg, 100.0f)
                                  : 0);
  frame.data[4] = static_cast<std::uint8_t>(
      (state.lateral_valid ? 1u << 0 : 0u) |
      (state.heading_valid ? 1u << 1 : 0u) |
      (state.front_pair_valid ? 1u << 2 : 0u) |
      (state.rear_pair_valid ? 1u << 3 : 0u) |
      (state.estimated ? 1u << 4 : 0u) |
      (state.sensor_fault ? 1u << 5 : 0u) |
      (state.calibration_valid ? 1u << 6 : 0u) |
      (state.simulation_mode ? 1u << 7 : 0u));
  frame.data[5] = state.counter;
  return frame;
}

bool decodePose(const CanFrame& frame, CanPoseState& state) {
  if (frame.id != static_cast<std::uint16_t>(CanId::Pose) || frame.dlc != 8) {
    return false;
  }
  state.lateral_offset_m = readSigned(frame.data, 0) / 10000.0f;
  state.heading_deg = readSigned(frame.data, 2) / 100.0f;
  const std::uint8_t flags = frame.data[4];
  state.lateral_valid = (flags & (1u << 0)) != 0;
  state.heading_valid = (flags & (1u << 1)) != 0;
  state.front_pair_valid = (flags & (1u << 2)) != 0;
  state.rear_pair_valid = (flags & (1u << 3)) != 0;
  state.estimated = (flags & (1u << 4)) != 0;
  state.sensor_fault = (flags & (1u << 5)) != 0;
  state.calibration_valid = (flags & (1u << 6)) != 0;
  state.simulation_mode = (flags & (1u << 7)) != 0;
  state.counter = frame.data[5];
  return true;
}

CanFrame encodeArms(CanId id, const CanArmState& state) {
  if (id != CanId::FrontArms && id != CanId::RearArms) {
    return {};
  }
  CanFrame frame = emptyFrame(id);
  writeSigned(frame.data, 0, scaledSigned(state.left_angle_deg, 100.0f));
  writeSigned(frame.data, 2, scaledSigned(state.right_angle_deg, 100.0f));
  frame.data[4] = static_cast<std::uint8_t>(
      (state.left_contact ? 1u << 0 : 0u) |
      (state.right_contact ? 1u << 1 : 0u) |
      (state.pair_valid ? 1u << 2 : 0u) |
      (state.left_fault ? 1u << 3 : 0u) |
      (state.right_fault ? 1u << 4 : 0u) |
      (state.estimated ? 1u << 5 : 0u) |
      ((static_cast<std::uint8_t>(state.invalid_reason) & 0x0Fu) << 6));
  frame.data[5] = static_cast<std::uint8_t>(state.invalid_reason) >> 2;
  frame.data[6] = state.counter;
  return frame;
}

bool decodeArms(const CanFrame& frame, CanId id, CanArmState& state) {
  if ((id != CanId::FrontArms && id != CanId::RearArms) ||
      frame.id != static_cast<std::uint16_t>(id) || frame.dlc != 8) {
    return false;
  }
  state.left_angle_deg = readSigned(frame.data, 0) / 100.0f;
  state.right_angle_deg = readSigned(frame.data, 2) / 100.0f;
  const std::uint8_t flags = frame.data[4];
  state.left_contact = (flags & (1u << 0)) != 0;
  state.right_contact = (flags & (1u << 1)) != 0;
  state.pair_valid = (flags & (1u << 2)) != 0;
  state.left_fault = (flags & (1u << 3)) != 0;
  state.right_fault = (flags & (1u << 4)) != 0;
  state.estimated = (flags & (1u << 5)) != 0;
  const std::uint8_t reason = static_cast<std::uint8_t>(
      ((flags >> 6) & 0x03u) | ((frame.data[5] & 0x03u) << 2));
  state.invalid_reason = static_cast<CanInvalidReason>(reason);
  state.counter = frame.data[6];
  return true;
}

CanFrame encodeStatus(const CanStatusState& state) {
  CanFrame frame = emptyFrame(CanId::Status);
  const std::uint8_t enabled_pairs =
      (state.front_pair_enabled ? 1u : 0u) |
      (state.rear_pair_enabled ? 2u : 0u);
  frame.data[0] = static_cast<std::uint8_t>(
      (state.simulation_mode ? 1u : 0u) |
      ((enabled_pairs & 0x03u) << 1) |
      ((state.estimator & 0x03u) << 3) |
      ((state.quality & 0x03u) << 5) |
      (state.calibration_valid ? 1u << 7 : 0u));
  return frame;
}

bool decodeStatus(const CanFrame& frame, CanStatusState& state) {
  if (frame.id != static_cast<std::uint16_t>(CanId::Status) || frame.dlc != 8) {
    return false;
  }
  const std::uint8_t flags = frame.data[0];
  state.simulation_mode = (flags & 0x01u) != 0;
  const std::uint8_t enabled_pairs = (flags >> 1) & 0x03u;
  state.front_pair_enabled = (enabled_pairs & 0x01u) != 0;
  state.rear_pair_enabled = (enabled_pairs & 0x02u) != 0;
  state.estimator = (flags >> 3) & 0x03u;
  state.quality = (flags >> 5) & 0x03u;
  state.calibration_valid = (flags & 0x80u) != 0;
  return true;
}

CanFrame encodeHeartbeat(std::uint8_t counter, std::uint8_t major,
                         std::uint8_t minor, std::uint8_t patch) {
  CanFrame frame = emptyFrame(CanId::Heartbeat);
  frame.data[0] = counter;
  frame.data[1] = major;
  frame.data[2] = minor;
  frame.data[3] = patch;
  return frame;
}

bool decodeHeartbeat(const CanFrame& frame, CanFirmwareVersion& version) {
  if (frame.id != static_cast<std::uint16_t>(CanId::Heartbeat) || frame.dlc != 8) {
    return false;
  }
  version.counter = frame.data[0];
  version.major = frame.data[1];
  version.minor = frame.data[2];
  version.patch = frame.data[3];
  return true;
}

CanFrame encodeOffsets(const CanOffsets& offsets) {
  CanFrame frame = emptyFrame(CanId::Offsets);
  writeSigned(frame.data, 0, scaledSigned(offsets.rear_axle_m, 10000.0f));
  writeSigned(frame.data, 2,
              scaledSigned(offsets.wheelbase_center_m, 10000.0f));
  writeSigned(frame.data, 4, scaledSigned(offsets.front_axle_m, 10000.0f));
  frame.data[6] = offsets.counter;
  return frame;
}

bool decodeOffsets(const CanFrame& frame, CanOffsets& offsets) {
  if (frame.id != static_cast<std::uint16_t>(CanId::Offsets) || frame.dlc != 8) {
    return false;
  }
  offsets.rear_axle_m = readSigned(frame.data, 0) / 10000.0f;
  offsets.wheelbase_center_m = readSigned(frame.data, 2) / 10000.0f;
  offsets.front_axle_m = readSigned(frame.data, 4) / 10000.0f;
  offsets.counter = frame.data[6];
  return true;
}

}  // namespace palpeur