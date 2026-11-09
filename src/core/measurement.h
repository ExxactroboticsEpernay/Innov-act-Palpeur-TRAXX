#pragma once

namespace palpeur {

enum class InvalidReason {
  None,
  LeftFree,
  RightFree,
  BothFree,
  LeftSensorFault,
  RightSensorFault,
  BothSensorFault,
  CalibrationInvalid,
  GeometryInvalid,
  PairDisabled,  // paire désactivée dans la configuration (pas une panne)
  NoRecentVine   // aucun passage de cep mesuré récemment (arrêt, trou dans le rang)
};

struct PairConfig {
  float x_m = 0.0f;
  float pivot_spacing_m = 0.515f;   // mesuré sur table (2026-11-09)
  float contact_length_m = 0.395f;  // = L mesurée
  int arm_direction = 1;
};

struct PairInput {
  float left_angle_deg = 0.0f;
  float right_angle_deg = 0.0f;
  bool left_contact = false;
  bool right_contact = false;
  bool left_sensor_valid = true;
  bool right_sensor_valid = true;
  bool calibration_valid = true;
};

struct PairEstimate {
  bool valid = false;
  bool estimated = false;
  InvalidReason reason = InvalidReason::None;
  float x_m = 0.0f;
  float row_y_m = 0.0f;
  float apparent_width_m = 0.0f;
};

struct VehicleGeometry {
  float wheelbase_m = 2.14f;  // TRAXX vignes étroites (fiche constructeur)
  float rear_pair_offset_m = -0.40f;
  float front_pair_offset_m = 0.40f;
};

struct RowPose {
  bool lateral_valid = false;
  bool heading_valid = false;
  float heading_deg = 0.0f;
  float offset_rear_axle_m = 0.0f;
  float offset_wheelbase_center_m = 0.0f;
  float offset_front_axle_m = 0.0f;
  float offset_at_valid_pair_m = 0.0f;
  float valid_pair_x_m = 0.0f;
  InvalidReason reason = InvalidReason::None;
};

}  // namespace palpeur