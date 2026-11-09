#include "geometry.h"

#include <cmath>

namespace palpeur {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinSpacingM = 0.05f;
constexpr float kMaxSpacingM = 2.0f;
constexpr float kMinContactLengthM = 0.01f;
constexpr float kMaxContactLengthM = 2.0f;
constexpr float kMinPairSeparationM = 0.1f;

struct Vector2 {
  float x;
  float y;
};

float cross(const Vector2& left, const Vector2& right) {
  return left.x * right.y - left.y * right.x;
}

Vector2 subtract(const Vector2& left, const Vector2& right) {
  return {left.x - right.x, left.y - right.y};
}

float interpolate(float y0, float y1, float x0, float x1, float x) {
  return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

bool validConfig(const PairConfig& config) {
  return std::isfinite(config.x_m) &&
         std::isfinite(config.pivot_spacing_m) &&
         std::isfinite(config.contact_length_m) &&
         config.pivot_spacing_m >= kMinSpacingM &&
         config.pivot_spacing_m <= kMaxSpacingM &&
         config.contact_length_m >= kMinContactLengthM &&
         config.contact_length_m <= kMaxContactLengthM &&
         (config.arm_direction == -1 || config.arm_direction == 1);
}

InvalidReason inputReason(const PairInput& input) {
  if (!input.calibration_valid) return InvalidReason::CalibrationInvalid;
  if (!input.left_sensor_valid && !input.right_sensor_valid)
    return InvalidReason::BothSensorFault;
  if (!input.left_sensor_valid) return InvalidReason::LeftSensorFault;
  if (!input.right_sensor_valid) return InvalidReason::RightSensorFault;
  if (!input.left_contact && !input.right_contact) return InvalidReason::BothFree;
  if (!input.left_contact) return InvalidReason::LeftFree;
  if (!input.right_contact) return InvalidReason::RightFree;
  return InvalidReason::None;
}

PairEstimate invalidEstimate(const PairConfig& config,
                             const PairInput& input) {
  PairEstimate result;
  result.x_m = config.x_m;
  result.reason = inputReason(input);
  if (result.reason == InvalidReason::None) {
    result.reason = InvalidReason::GeometryInvalid;
  }
  return result;
}

bool contactPoints(const PairConfig& config, const PairInput& input,
                   Vector2& left_point, Vector2& right_point,
                   Vector2& left_direction, Vector2& right_direction) {
  const float left_radians = input.left_angle_deg * kPi / 180.0f;
  const float right_radians = input.right_angle_deg * kPi / 180.0f;
  if (!std::isfinite(left_radians) || !std::isfinite(right_radians)) {
    return false;
  }

  left_direction = {config.arm_direction * std::cos(left_radians),
                    -std::sin(left_radians)};
  right_direction = {config.arm_direction * std::cos(right_radians),
                     std::sin(right_radians)};
  const Vector2 left_pivot = {config.x_m, config.pivot_spacing_m / 2.0f};
  const Vector2 right_pivot = {config.x_m, -config.pivot_spacing_m / 2.0f};
  left_point = {left_pivot.x + config.contact_length_m * left_direction.x,
                left_pivot.y + config.contact_length_m * left_direction.y};
  right_point = {right_pivot.x + config.contact_length_m * right_direction.x,
                 right_pivot.y + config.contact_length_m * right_direction.y};
  return std::isfinite(left_point.x) && std::isfinite(left_point.y) &&
         std::isfinite(right_point.x) && std::isfinite(right_point.y);
}

PairEstimate checkedInput(const PairConfig& config, const PairInput& input) {
  if (!validConfig(config)) {
    PairEstimate result;
    result.x_m = config.x_m;
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }
  if (inputReason(input) != InvalidReason::None) {
    return invalidEstimate(config, input);
  }
  return {};
}

}  // namespace

PairEstimate ContactPointEstimator::estimate(const PairConfig& config,
                                              const PairInput& input) const {
  PairEstimate result = checkedInput(config, input);
  if (result.reason != InvalidReason::None) return result;

  Vector2 left_point{};
  Vector2 right_point{};
  Vector2 left_direction{};
  Vector2 right_direction{};
  if (!contactPoints(config, input, left_point, right_point,
                     left_direction, right_direction)) {
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }

  result.valid = true;
  result.row_y_m = (left_point.y + right_point.y) / 2.0f;
  result.apparent_width_m = std::fabs(left_point.y - right_point.y);
  return result;
}

PairEstimate BisectorEstimator::estimate(const PairConfig& config,
                                         const PairInput& input) const {
  PairEstimate result = checkedInput(config, input);
  if (result.reason != InvalidReason::None) return result;

  Vector2 left_contact{};
  Vector2 right_contact{};
  Vector2 left_direction{};
  Vector2 right_direction{};
  if (!contactPoints(config, input, left_contact, right_contact,
                     left_direction, right_direction)) {
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }

  const Vector2 left_pivot = {config.x_m, config.pivot_spacing_m / 2.0f};
  const Vector2 right_pivot = {config.x_m, -config.pivot_spacing_m / 2.0f};
  const Vector2 between_pivots = subtract(right_pivot, left_pivot);
  const float denominator = cross(left_direction, right_direction);
  if (std::fabs(denominator) < 1e-5f) {
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }

  const float left_parameter = cross(between_pivots, right_direction) /
                               denominator;
  const Vector2 apex = {
      left_pivot.x + left_parameter * left_direction.x,
      left_pivot.y + left_parameter * left_direction.y};

  const Vector2 bisector_sum = {left_direction.x + right_direction.x,
                                left_direction.y + right_direction.y};
  const float bisector_length = std::hypot(bisector_sum.x, bisector_sum.y);
  if (bisector_length < 1e-5f) {
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }

  const Vector2 bisector = {-bisector_sum.x / bisector_length,
                            -bisector_sum.y / bisector_length};
  if (std::fabs(bisector.x) < 1e-5f) {
    result.reason = InvalidReason::GeometryInvalid;
    return result;
  }

  const float station_parameter = (config.x_m - apex.x) / bisector.x;
  result.valid = true;
  result.row_y_m = apex.y + station_parameter * bisector.y;
  result.apparent_width_m = std::fabs(left_contact.y - right_contact.y);
  return result;
}

RowPose estimateRowPose(const VehicleGeometry& geometry,
                        const PairEstimate& rear,
                        const PairEstimate& front) {
  RowPose result;
  if (rear.valid && front.valid) {
    const float rear_x = geometry.rear_pair_offset_m;
    const float front_x = geometry.wheelbase_m + geometry.front_pair_offset_m;
    const float separation = front_x - rear_x;
    if (!std::isfinite(separation) || separation < kMinPairSeparationM) {
      result.reason = InvalidReason::GeometryInvalid;
      return result;
    }

    const float lateral_delta = front.row_y_m - rear.row_y_m;
    result.heading_deg = -std::atan2(lateral_delta, separation) * 180.0f / kPi;
    result.heading_valid = true;
    result.lateral_valid = true;
    result.offset_rear_axle_m = -interpolate(rear.row_y_m, front.row_y_m,
                                              rear_x, front_x, 0.0f);
    result.offset_wheelbase_center_m = -interpolate(
        rear.row_y_m, front.row_y_m, rear_x, front_x,
        geometry.wheelbase_m / 2.0f);
    result.offset_front_axle_m = -interpolate(
        rear.row_y_m, front.row_y_m, rear_x, front_x,
        geometry.wheelbase_m);
    return result;
  }

  if (front.valid || rear.valid) {
    const PairEstimate& valid_pair = front.valid ? front : rear;
    result.lateral_valid = true;
    result.offset_at_valid_pair_m = -valid_pair.row_y_m;
    result.valid_pair_x_m = valid_pair.x_m;
    return result;
  }

  result.reason = rear.reason != InvalidReason::None ? rear.reason : front.reason;
  return result;
}

}  // namespace palpeur