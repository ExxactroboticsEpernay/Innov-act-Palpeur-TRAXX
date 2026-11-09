#include "geometry_simulation.h"

#include <algorithm>
#include <cmath>

namespace palpeur {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct Vector2 {
  float x;
  float y;
};

bool tangentDirection(const Vector2& pivot, const Vector2& center,
                      float object_radius, int arm_direction, bool upper_contact,
                      Vector2& direction) {
  const float delta_x = center.x - pivot.x;
  const float delta_y = center.y - pivot.y;
  const float distance = std::hypot(delta_x, delta_y);
  if (!std::isfinite(distance) || !std::isfinite(object_radius) ||
      distance <= object_radius || object_radius <= 0.0f) {
    return false;
  }

  const float bearing = std::atan2(delta_y, delta_x);
  const float tangent_angle = std::asin(object_radius / distance);
  const float candidates[] = {bearing + tangent_angle,
                              bearing - tangent_angle};
  float best_score = -1.0f;
  bool found = false;
  for (float angle : candidates) {
    const Vector2 candidate = {std::cos(angle), std::sin(angle)};
    if (candidate.x * arm_direction <= 1e-4f) continue;
    const float projection = delta_x * candidate.x + delta_y * candidate.y;
    if (projection <= 0.0f) continue;
    const float contact_y = pivot.y + projection * candidate.y;
    const float score = upper_contact ? contact_y - center.y
                                      : center.y - contact_y;
    if (score > best_score) {
      best_score = score;
      direction = candidate;
      found = true;
    }
  }
  return found;
}

float clampUnit(float value) {
  return std::max(-1.0f, std::min(value, 1.0f));
}

}  // namespace

SimulatedContact simulateCircularObjectContact(
    const CircularObjectScenario& scenario) {
  SimulatedContact result;
  const float radius = scenario.diameter_m / 2.0f;
  if (!std::isfinite(scenario.diameter_m) || radius <= 0.0f ||
      !std::isfinite(scenario.row_y_at_pair_m) ||
      !std::isfinite(scenario.row_slope) ||
      (scenario.pair.arm_direction != -1 &&
       scenario.pair.arm_direction != 1) ||
      scenario.pair.pivot_spacing_m <= 0.0f ||
      scenario.pair.contact_length_m <= 0.0f) {
    return result;
  }

  const float station_x = scenario.pair.x_m;
  result.object_center_x_m = station_x +
      scenario.pair.arm_direction * scenario.pair.contact_length_m;
  result.object_center_y_m = scenario.row_y_at_pair_m +
      scenario.row_slope * (result.object_center_x_m - station_x);
  const Vector2 center = {result.object_center_x_m, result.object_center_y_m};
  const Vector2 left_pivot = {
      station_x, scenario.pair.pivot_spacing_m / 2.0f};
  const Vector2 right_pivot = {
      station_x, -scenario.pair.pivot_spacing_m / 2.0f};

  Vector2 left_direction{};
  Vector2 right_direction{};
  if (!tangentDirection(left_pivot, center,
                        radius, scenario.pair.arm_direction, true,
                        left_direction) ||
      !tangentDirection(right_pivot, center,
                        radius, scenario.pair.arm_direction, false,
                        right_direction)) {
    return result;
  }

  result.input.left_angle_deg =
      std::asin(clampUnit(-left_direction.y)) * 180.0f / kPi;
  result.input.right_angle_deg =
      std::asin(clampUnit(right_direction.y)) * 180.0f / kPi;
  result.input.left_contact = true;
  result.input.right_contact = true;
  result.valid = std::isfinite(result.input.left_angle_deg) &&
                 std::isfinite(result.input.right_angle_deg);
  return result;
}

}  // namespace palpeur