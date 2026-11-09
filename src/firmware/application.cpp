#include "application.h"

#include "board_io.h"
#include "../core/calibration.h"
#include "../core/geometry.h"
#include "../core/sensor_pipeline.h"
#include "../core/vine_events.h"
#include "../sim/geometry_simulation.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <algorithm>
#include <LittleFS.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

#include <cmath>
#include <array>
#include <cstdio>
#include <cstring>

#ifndef PALPEUR_WIFI_SSID
#define PALPEUR_WIFI_SSID "PALPEUR-TRAXX"
#endif

#ifndef PALPEUR_WIFI_PASSWORD
#define PALPEUR_WIFI_PASSWORD "palpeur-dev"
#endif

static_assert(sizeof(PALPEUR_WIFI_PASSWORD) - 1 >= 8,
              "WPA2 requires a Wi-Fi password of at least 8 characters");

namespace palpeur {
namespace firmware {
namespace {

// Paires : 0 = avant, 1 = arrière. Capteurs : paire * 2 + côté (0 gauche, 1 droit),
// soit le même ordre que board_io.h.
constexpr int kPairCount = 2;
constexpr int kFront = 0;
constexpr int kRear = 1;
constexpr int kLeft = 0;
constexpr int kRight = 1;
const char* const kSensorNames[kSensorChannelCount] = {
    "front_left", "front_right", "rear_left", "rear_right"};
// Préfixes NVS courts : les clés Preferences sont limitées à 15 caractères.
const char* const kSensorKeys[kSensorChannelCount] = {"fl", "fr", "rl", "rr"};

int sensorOf(int pair, int side) { return pair * 2 + side; }

struct AppConfig {
  float wheelbase_m = 2.14f;
  float pivot_spacing_m = 0.515f;
  float arm_length_m = 0.395f;
  float contact_length_m = 0.395f;
  // Décalage signé du pivot par rapport à l'essieu correspondant.
  float pair_offset_m[kPairCount] = {0.40f, -0.40f};
  // Angle géométrique des bras au repos, mesuré depuis l'axe x (avant),
  // positif vers l'intérieur. 90° = bras perpendiculaires à la machine.
  // Angle du bras = angle de repos + balayage capteur (positif vers l'arrière).
  float rest_angle_deg[kPairCount] = {90.0f, 90.0f};
  bool pair_enabled[kPairCount] = {true, false};
  // Biais de centrage par paire, retranché à y_row (calibration « centre »).
  float center_bias_m[kPairCount] = {0.0f, 0.0f};
  // Écart G/D (φG − φD) mesuré machine centrée : zéro de la comparaison G/D.
  float center_lr_diff_deg[kPairCount] = {0.0f, 0.0f};
  float track_width_m = 1.16f;
  float contact_threshold_deg = 3.0f;
  float contact_hysteresis_deg = 1.0f;
  float smoothing_alpha = 0.25f;
  float maximum_slew_deg_per_second = 300.0f;
  float object_diameter_m = 0.10f;
  int median_window = 5;
  int publish_rate_hz = 50;
  int estimator = 1;
};

// Acquisition : à 6 km/h un passage de cep dure ~180 ms ; 250 Hz donne ~45
// échantillons par passage (50 Hz n'en donnait que 9, mangés par la médiane
// et la confirmation). La publication (page, CAN) reste à sa propre cadence.
constexpr std::uint32_t kSampleRateHz = 250;
// Une mesure de cep reste utilisée pour la pose pendant ce temps : à 6 km/h
// un cep passe toutes les 0,6 s, à 1 km/h toutes les 3,6 s.
constexpr float kVineHoldS = 5.0f;
constexpr std::size_t kCalibrationWindowSize = 2 * kSampleRateHz;  // 2 s

struct SensorState {
  SensorCalibration calibration;
  bool neutral_captured = false;
  bool endpoints_captured = false;
  bool direction_captured = false;  // sens « vers l'arrière » appris
  float capture_min_v = 4.7f;
  float capture_max_v = 0.3f;
  std::array<float, kCalibrationWindowSize> history{};
  SensorChannel* channel = nullptr;
  SensorChannelReading reading;
};

struct RuntimeState {
  PairEstimate pairs[kPairCount];
  float raw_row_y_m[kPairCount] = {0.0f, 0.0f};  // avant correction du biais
  // Comparaison G/D brute (φG − φD), valable seulement si les deux bras de la
  // paire touchent le même cep au même instant.
  float raw_lr_diff_deg[kPairCount] = {NAN, NAN};
  // Mesure par cep et estimation « tenue » (dernier cep récent) utilisée pour
  // la pose : les paires AV et AR touchent rarement un cep au même instant.
  VineEventTracker vines[kPairCount];
  PairEstimate held[kPairCount];
  RowPose pose;
  std::uint32_t sequence = 0;
  bool simulated = PALPEUR_SIMULATION != 0;
};

WebServer web_server(80);
Preferences preferences;
AppConfig app_config;
RuntimeState runtime_state;
SensorState sensors[kSensorChannelCount];
bool center_captured[kPairCount] = {false, false};
std::uint32_t last_sample_ms = 0;
std::uint32_t ota_restart_at_ms = 0;
float simulation_phase = 0.0f;
std::size_t voltage_history_next = 0;
std::size_t voltage_history_count = 0;
bool endpoint_capture_active = false;
std::uint32_t endpoint_capture_started_ms = 0;

const char* sensorKey(int sensor, const char* suffix) {
  static char key[16];
  std::snprintf(key, sizeof(key), "%s_%s", kSensorKeys[sensor], suffix);
  return key;
}

bool sensorEnabled(int sensor) { return app_config.pair_enabled[sensor / 2]; }

const char* activePairsName() {
  if (app_config.pair_enabled[kFront] && app_config.pair_enabled[kRear]) return "both";
  return app_config.pair_enabled[kRear] ? "rear" : "front";
}

const char* reasonName(InvalidReason reason) {
  switch (reason) {
    case InvalidReason::None: return "none";
    case InvalidReason::LeftFree: return "left_free";
    case InvalidReason::RightFree: return "right_free";
    case InvalidReason::BothFree: return "both_free";
    case InvalidReason::LeftSensorFault: return "left_sensor_fault";
    case InvalidReason::RightSensorFault: return "right_sensor_fault";
    case InvalidReason::BothSensorFault: return "both_sensor_fault";
    case InvalidReason::CalibrationInvalid: return "calibration_invalid";
    case InvalidReason::GeometryInvalid: return "geometry_invalid";
    case InvalidReason::PairDisabled: return "pair_disabled";
    case InvalidReason::NoRecentVine: return "no_recent_vine";
  }
  return "unknown";
}

// ArduinoJson écrirait NaN tel quel (JSON invalide) : on publie null à la place.
void setNumber(JsonObject object, const char* key, float value) {
  if (std::isfinite(value)) {
    object[key] = value;
  } else {
    object[key] = nullptr;
  }
}

PairConfig pairConfig(int pair) {
  PairConfig config;
  config.x_m = pair == kFront
      ? app_config.wheelbase_m + app_config.pair_offset_m[kFront]
      : app_config.pair_offset_m[kRear];
  config.pivot_spacing_m = app_config.pivot_spacing_m;
  config.contact_length_m = app_config.contact_length_m;
  config.arm_direction = 1;  // angles géométriques toujours mesurés depuis +x
  return config;
}

VehicleGeometry vehicleGeometry() {
  VehicleGeometry geometry;
  geometry.wheelbase_m = app_config.wheelbase_m;
  geometry.front_pair_offset_m = app_config.pair_offset_m[kFront];
  geometry.rear_pair_offset_m = app_config.pair_offset_m[kRear];
  return geometry;
}

SensorChannelConfig channelConfig(const SensorCalibration& calibration) {
  SensorChannelConfig config;
  config.calibration = calibration;
  config.contact_threshold_deg = app_config.contact_threshold_deg;
  config.contact_hysteresis_deg = app_config.contact_hysteresis_deg;
  config.smoothing_alpha = app_config.smoothing_alpha;
  config.maximum_slew_deg_per_second = app_config.maximum_slew_deg_per_second;
  config.median_window = static_cast<std::size_t>(app_config.median_window);
  return config;
}

void recreateChannels() {
  for (SensorState& sensor : sensors) {
    delete sensor.channel;
    sensor.channel = new SensorChannel(channelConfig(sensor.calibration));
    sensor.reading = SensorChannelReading{};
  }
}

void loadConfig() {
  app_config.wheelbase_m = preferences.getFloat("wheelbase", 2.14f);
  app_config.pivot_spacing_m = preferences.getFloat("spacing", 0.515f);
  app_config.arm_length_m = preferences.getFloat("arm_len", 0.395f);
  app_config.contact_length_m = preferences.getFloat("contact", 0.395f);
  app_config.pair_offset_m[kFront] = preferences.getFloat("front_off", 0.40f);
  app_config.pair_offset_m[kRear] = preferences.getFloat("rear_off", -0.40f);
  app_config.rest_angle_deg[kFront] = preferences.getFloat("rest_ang", 90.0f);
  app_config.rest_angle_deg[kRear] = preferences.getFloat("rest_ang_r", 90.0f);
  app_config.pair_enabled[kFront] = preferences.getBool("front_en", true);
  app_config.pair_enabled[kRear] = preferences.getBool("rear_en", false);
  if (!app_config.pair_enabled[kFront] && !app_config.pair_enabled[kRear]) {
    app_config.pair_enabled[kFront] = true;  // au moins une paire active
  }
  app_config.center_bias_m[kFront] = preferences.getFloat("center", 0.0f);
  app_config.center_bias_m[kRear] = preferences.getFloat("center_r", 0.0f);
  app_config.center_lr_diff_deg[kFront] = preferences.getFloat("cdiff", 0.0f);
  app_config.center_lr_diff_deg[kRear] = preferences.getFloat("cdiff_r", 0.0f);
  app_config.track_width_m = preferences.getFloat("track", 1.16f);
  app_config.contact_threshold_deg = preferences.getFloat("threshold", 3.0f);
  app_config.contact_hysteresis_deg = preferences.getFloat("hyst", 1.0f);
  app_config.smoothing_alpha = preferences.getFloat("smooth", 0.25f);
  app_config.maximum_slew_deg_per_second = preferences.getFloat("slew", 300.0f);
  app_config.object_diameter_m = preferences.getFloat("diameter", 0.10f);
  app_config.median_window = preferences.getInt("median", 5);
  app_config.publish_rate_hz = preferences.getInt("rate", 50);
  app_config.estimator = preferences.getInt("estimator", 1);
  center_captured[kFront] = preferences.getBool("center_set", false);
  center_captured[kRear] = preferences.getBool("center_set_r", false);

  for (int index = 0; index < kSensorChannelCount; ++index) {
    SensorState& sensor = sensors[index];
    sensor.calibration.voltage_min_v = preferences.getFloat(sensorKey(index, "min"), 0.5f);
    sensor.calibration.voltage_neutral_v = preferences.getFloat(sensorKey(index, "neu"), 2.5f);
    sensor.calibration.voltage_max_v = preferences.getFloat(sensorKey(index, "max"), 4.5f);
    sensor.calibration.invert = preferences.getBool(sensorKey(index, "inv"), false);
    sensor.neutral_captured = preferences.getBool(sensorKey(index, "nset"), false);
    sensor.endpoints_captured = preferences.getBool(sensorKey(index, "eset"), false);
    sensor.direction_captured = preferences.getBool(sensorKey(index, "dset"), false);
#if PALPEUR_SIMULATION
    sensor.calibration.calibrated = true;
#else
    sensor.calibration.calibrated = preferences.getBool(sensorKey(index, "ok"), false);
#endif
  }
}

bool validNumber(JsonVariantConst value, float minimum, float maximum,
                 float& output) {
  if (!value.is<float>() && !value.is<int>()) return false;
  output = value.as<float>();
  return std::isfinite(output) && output >= minimum && output <= maximum;
}

void persistCalibration() {
  for (int index = 0; index < kSensorChannelCount; ++index) {
    const SensorState& sensor = sensors[index];
    preferences.putFloat(sensorKey(index, "min"), sensor.calibration.voltage_min_v);
    preferences.putFloat(sensorKey(index, "neu"), sensor.calibration.voltage_neutral_v);
    preferences.putFloat(sensorKey(index, "max"), sensor.calibration.voltage_max_v);
    preferences.putBool(sensorKey(index, "nset"), sensor.neutral_captured);
    preferences.putBool(sensorKey(index, "eset"), sensor.endpoints_captured);
    preferences.putBool(sensorKey(index, "dset"), sensor.direction_captured);
    preferences.putBool(sensorKey(index, "inv"), sensor.calibration.invert);
  }
  preferences.putBool("center_set", center_captured[kFront]);
  preferences.putBool("center_set_r", center_captured[kRear]);
}

void updateCalibrationValidity() {
  for (int index = 0; index < kSensorChannelCount; ++index) {
    SensorState& sensor = sensors[index];
#if PALPEUR_SIMULATION
    sensor.calibration.calibrated = true;
#else
    SensorCalibration check = sensor.calibration;
    check.calibrated = true;
    sensor.calibration.calibrated = sensor.neutral_captured &&
                                    sensor.direction_captured &&
                                    sensor.endpoints_captured &&
                                    isCalibrationValid(check);
    preferences.putBool(sensorKey(index, "ok"), sensor.calibration.calibrated);
#endif
  }
  recreateChannels();
}

// Calibration complète = tous les capteurs des paires actives calibrés.
bool calibrationComplete() {
  for (int index = 0; index < kSensorChannelCount; ++index) {
    if (!sensorEnabled(index)) continue;
    const SensorState& sensor = sensors[index];
    if (!sensor.neutral_captured || !sensor.direction_captured ||
        !sensor.endpoints_captured || !sensor.calibration.calibrated) {
      return false;
    }
  }
  return true;
}

void finishEndpointCapture() {
  endpoint_capture_active = false;
  for (int index = 0; index < kSensorChannelCount; ++index) {
    if (!sensorEnabled(index)) continue;  // paire inactive : calibration conservée
    SensorState& sensor = sensors[index];
    // Contrôle de cohérence : au moins 1 V (≈ 45°) de course balayée.
    sensor.endpoints_captured = sensor.capture_max_v - sensor.capture_min_v >= 1.0f;
    if (sensor.endpoints_captured) {
      sensor.calibration.voltage_min_v = sensor.capture_min_v;
      sensor.calibration.voltage_max_v = sensor.capture_max_v;
    }
  }
  updateCalibrationValidity();
  persistCalibration();
}

void recordCalibrationSample(const float (&voltages)[kSensorChannelCount],
                             std::uint32_t now) {
  for (int index = 0; index < kSensorChannelCount; ++index) {
    SensorState& sensor = sensors[index];
    const float voltage = voltages[index];
    sensor.history[voltage_history_next] = voltage;
    if (endpoint_capture_active && voltage >= 0.3f && voltage <= 4.7f) {
      sensor.capture_min_v = std::min(sensor.capture_min_v, voltage);
      sensor.capture_max_v = std::max(sensor.capture_max_v, voltage);
    }
  }
  voltage_history_next = (voltage_history_next + 1) % kCalibrationWindowSize;
  if (voltage_history_count < kCalibrationWindowSize) ++voltage_history_count;
  if (endpoint_capture_active && now - endpoint_capture_started_ms >= 10000) {
    finishEndpointCapture();
  }
}

void sendJson(JsonDocument& document, int status = 200) {
  String body;
  serializeJson(document, body);
  web_server.send(status, "application/json; charset=utf-8", body);
}

void sendError(int status, const char* message) {
  StaticJsonDocument<256> response;
  response["ok"] = false;
  response["error"] = message;
  sendJson(response, status);
}

// Simulation : tension d'un capteur au neutre 2,5 V, sens non inversé.
float simulatedVoltageFromSweep(float sweep_deg) {
  return 2.5f + sweep_deg * kNominalVoltsPerDegree;
}

float armAngleDeg(int pair, float sweep_deg) {
  return app_config.rest_angle_deg[pair] + sweep_deg;
}

void updateMeasurements() {
  const std::uint32_t now = millis();
  const float delta_seconds = last_sample_ms == 0
      ? 1.0f / kSampleRateHz
      : static_cast<float>(now - last_sample_ms) / 1000.0f;
  last_sample_ms = now;

  float voltages[kSensorChannelCount];
  for (float& voltage : voltages) voltage = NAN;

#if PALPEUR_SIMULATION
  // Rang réel : un cep tous les 1 m, un piquet tous les 7 m (entre deux
  // ceps), TRAXX à 6 km/h. La machine serpente lentement : décalage ±3,5 cm,
  // lacet ±0,7°. Chaque objet repousse les bras vers l'arrière puis les
  // libère ; les ressorts les ramènent avec dépassement et rebonds.
  constexpr float kSimSpeedMps = 6.0f / 3.6f;
  constexpr float kVineSpacingM = 1.0f;
  constexpr float kStakeSpacingM = 7.0f;
  constexpr float kStakeOffsetM = 0.5f;
  constexpr float kStakeDiameterM = 0.06f;
  simulation_phase += delta_seconds;
  const float travel_m = kSimSpeedMps * simulation_phase;
  const float front_x = pairConfig(kFront).x_m;
  const float rear_x = pairConfig(kRear).x_m;
  const float middle_x = (front_x + rear_x) / 2.0f;
  const float row_y_at_middle_m = 0.035f * std::sin(simulation_phase * 0.32f);
  const float row_slope = std::tan(0.012f * std::sin(simulation_phase * 0.18f));
  for (int pair = 0; pair < kPairCount; ++pair) {
    if (!app_config.pair_enabled[pair]) continue;
    const PairConfig config = pairConfig(pair);
    // Dernier objet ayant atteint la ligne des bras au repos : q = distance
    // parcourue depuis le premier contact avec cet objet.
    const float s = travel_m + config.x_m;
    const float q_vine = s - kVineSpacingM * std::floor(s / kVineSpacingM);
    const float stake_s = s - kStakeOffsetM;
    const float q_stake = stake_s - kStakeSpacingM * std::floor(stake_s / kStakeSpacingM);
    const bool stake = q_stake < q_vine;
    const float q = stake ? q_stake : q_vine;
    const float diameter = stake ? kStakeDiameterM : app_config.object_diameter_m;

    // Angles atteints au moment où l'objet quitte le bout des bras.
    // Le simulateur (s = -1) mesure depuis -x : angle géométrique = 180° - θ_sim.
    CircularObjectScenario scenario;
    scenario.pair = config;
    scenario.pair.arm_direction = -1;
    scenario.diameter_m = diameter;
    scenario.row_y_at_pair_m = row_y_at_middle_m + row_slope * (config.x_m - middle_x);
    scenario.row_slope = row_slope;
    const SimulatedContact simulated = simulateCircularObjectContact(scenario);
    float sweep_left = 0.0f;
    float sweep_right = 0.0f;
    if (simulated.valid) {
      const float rest = app_config.rest_angle_deg[pair];
      const float release_left = 180.0f - simulated.input.left_angle_deg - rest;
      const float release_right = 180.0f - simulated.input.right_angle_deg - rest;
      const float mean_release_rad = (release_left + release_right) * 0.5f * 3.14159265f / 180.0f;
      // Trajet en contact ≈ projection du bras au relâchement + rayon de l'objet.
      const float contact_travel_m =
          app_config.arm_length_m * std::sin(mean_release_rad) + diameter / 2.0f;
      if (q <= contact_travel_m) {
        const float progress = q / contact_travel_m;
        sweep_left = release_left * progress;
        sweep_right = release_right * progress;
      } else {
        // Claquement : dépassement amorti autour du repos pendant ~150 ms.
        const float tau = (q - contact_travel_m) / kSimSpeedMps;
        if (tau < 0.15f) {
          const float snap = -6.0f * std::exp(-tau / 0.03f) *
                             std::cos(2.0f * 3.14159265f * tau / 0.05f);
          sweep_left = snap;
          sweep_right = snap;
        }
      }
    }
    voltages[sensorOf(pair, kLeft)] = simulatedVoltageFromSweep(sweep_left);
    voltages[sensorOf(pair, kRight)] = simulatedVoltageFromSweep(sweep_right);
  }
#else
  for (int index = 0; index < kSensorChannelCount; ++index) {
    if (sensorEnabled(index)) voltages[index] = readSensorVoltage(index);
  }
#endif

  recordCalibrationSample(voltages, now);

  for (int index = 0; index < kSensorChannelCount; ++index) {
    SensorState& sensor = sensors[index];
    if (!sensorEnabled(index)) {
      sensor.reading = SensorChannelReading{};
      sensor.channel->reset();
      continue;
    }
    sensor.reading = sensor.channel->update(voltages[index], delta_seconds);
  }

  for (int pair = 0; pair < kPairCount; ++pair) {
    const PairConfig config = pairConfig(pair);
    PairEstimate& estimate = runtime_state.pairs[pair];
    if (!app_config.pair_enabled[pair]) {
      estimate = PairEstimate{};
      estimate.x_m = config.x_m;
      estimate.reason = InvalidReason::PairDisabled;
      runtime_state.raw_lr_diff_deg[pair] = NAN;
      runtime_state.vines[pair].reset();
      runtime_state.held[pair] = estimate;
      continue;
    }
    const SensorState& left = sensors[sensorOf(pair, kLeft)];
    const SensorState& right = sensors[sensorOf(pair, kRight)];
    PairInput input;
    input.left_angle_deg = armAngleDeg(pair, left.reading.filtered_angle_deg);
    input.right_angle_deg = armAngleDeg(pair, right.reading.filtered_angle_deg);
    input.left_contact = left.reading.contact;
    input.right_contact = right.reading.contact;
    input.left_sensor_valid = left.reading.valid;
    input.right_sensor_valid = right.reading.valid;
    input.calibration_valid = left.calibration.calibrated &&
                              right.calibration.calibrated;
    estimate = app_config.estimator == 2
        ? BisectorEstimator().estimate(config, input)
        : ContactPointEstimator().estimate(config, input);
    runtime_state.raw_row_y_m[pair] = estimate.row_y_m;
    const bool both_in_contact = left.reading.valid && right.reading.valid &&
                                 left.reading.contact && right.reading.contact;
    runtime_state.raw_lr_diff_deg[pair] = both_in_contact
        ? left.reading.filtered_angle_deg - right.reading.filtered_angle_deg
        : NAN;
    if (estimate.valid) estimate.row_y_m -= app_config.center_bias_m[pair];

    // Mesure par cep : un passage = un double contact, résumé à sa fin.
    VineEventTracker& vine = runtime_state.vines[pair];
    vine.update(estimate.valid,
                runtime_state.raw_lr_diff_deg[pair] - app_config.center_lr_diff_deg[pair],
                estimate.row_y_m,
                (left.reading.filtered_angle_deg + right.reading.filtered_angle_deg) * 0.5f,
                delta_seconds);
    PairEstimate& held = runtime_state.held[pair];
    held = PairEstimate{};
    held.x_m = config.x_m;
    if (vine.fresh(kVineHoldS)) {
      held.valid = true;
      held.row_y_m = vine.last().row_y_m;
    } else {
      held.reason = estimate.valid || estimate.reason == InvalidReason::BothFree
          ? InvalidReason::NoRecentVine
          : estimate.reason;
    }
  }

  // Pose à partir du dernier cep récent de chaque paire.
  runtime_state.pose = estimateRowPose(vehicleGeometry(),
                                       runtime_state.held[kRear],
                                       runtime_state.held[kFront]);
  ++runtime_state.sequence;
}

// angle_raw_deg / angle_filtered_deg : balayage depuis le repos (+ = vers
// l'arrière). arm_angle_deg : angle géométrique depuis l'axe x (repos + balayage).
void fillSensorJson(JsonObject object, const SensorState& sensor, int pair) {
  const bool valid = sensor.reading.valid;
  setNumber(object, "voltage_v", sensor.reading.voltage_v);
  setNumber(object, "angle_raw_deg", valid ? sensor.reading.raw_angle_deg : NAN);
  setNumber(object, "angle_filtered_deg", valid ? sensor.reading.filtered_angle_deg : NAN);
  setNumber(object, "arm_angle_deg",
            valid ? armAngleDeg(pair, sensor.reading.filtered_angle_deg) : NAN);
  object["contact"] = sensor.reading.contact;
  object["fault"] = !sensor.reading.valid;
  object["rebound"] = sensor.reading.rebound;
}

void fillPairJson(JsonObject object, int pair) {
  const PairEstimate& estimate = runtime_state.pairs[pair];
  object["enabled"] = app_config.pair_enabled[pair];
  object["valid"] = estimate.valid;
  object["reason"] = reasonName(estimate.reason);
  setNumber(object, "x_m", estimate.x_m);
  setNumber(object, "row_y_m", estimate.valid ? estimate.row_y_m : NAN);
  // Comparaison G/D : > 0 = bras gauche plus repoussé = rang plus près du
  // pivot gauche = machine décalée à droite. Zéro fixé par la calibration centre.
  setNumber(object, "lr_diff_deg",
            runtime_state.raw_lr_diff_deg[pair] - app_config.center_lr_diff_deg[pair]);
  setNumber(object, "apparent_width_m",
            estimate.valid ? estimate.apparent_width_m : NAN);
  // Dernier passage de cep (ou piquet) : c'est la mesure à exploiter.
  const VineMeasurement& vine = runtime_state.vines[pair].last();
  if (vine.available) {
    JsonObject last = object["last_vine"].to<JsonObject>();
    last["lr_diff_deg"] = vine.lr_diff_deg;
    last["row_y_m"] = vine.row_y_m;
    last["peak_sweep_deg"] = vine.peak_sweep_deg;
    last["duration_s"] = vine.duration_s;
    last["age_s"] = vine.age_s;
    last["count"] = vine.count;
    last["fresh"] = vine.age_s <= kVineHoldS;
  } else {
    object["last_vine"] = nullptr;
  }
  fillSensorJson(object["left"].to<JsonObject>(), sensors[sensorOf(pair, kLeft)], pair);
  fillSensorJson(object["right"].to<JsonObject>(), sensors[sensorOf(pair, kRight)], pair);
}

void handleState() {
  DynamicJsonDocument document(3072);
  document["timestamp_ms"] = millis();
  document["sequence"] = runtime_state.sequence;
  document["simulation"] = runtime_state.simulated;
  document["active_pairs"] = activePairsName();
  fillPairJson(document["front_pair"].to<JsonObject>(), kFront);
  fillPairJson(document["rear_pair"].to<JsonObject>(), kRear);

  // Décalage principal : centre d'empattement si les deux paires sont valides
  // (cap disponible), sinon décalage local à la station de la paire valide.
  const RowPose& pose = runtime_state.pose;
  JsonObject pose_json = document["pose"].to<JsonObject>();
  pose_json["lateral_valid"] = pose.lateral_valid;
  pose_json["heading_valid"] = pose.heading_valid;
  pose_json["estimated"] = false;
  pose_json["reason"] = reasonName(pose.reason);
  if (pose.heading_valid) {
    pose_json["lateral_offset_m"] = pose.offset_wheelbase_center_m;
    pose_json["lateral_reference"] = "wheelbase_center";
    pose_json["lateral_reference_x_m"] = app_config.wheelbase_m / 2.0f;
    pose_json["heading_deg"] = pose.heading_deg;
    JsonObject offsets = pose_json["offsets"].to<JsonObject>();
    offsets["rear_axle_m"] = pose.offset_rear_axle_m;
    offsets["wheelbase_center_m"] = pose.offset_wheelbase_center_m;
    offsets["front_axle_m"] = pose.offset_front_axle_m;
  } else if (pose.lateral_valid) {
    const bool front_valid = runtime_state.held[kFront].valid;
    pose_json["lateral_offset_m"] = pose.offset_at_valid_pair_m;
    pose_json["lateral_reference"] = front_valid ? "front_pair" : "rear_pair";
    pose_json["lateral_reference_x_m"] = pose.valid_pair_x_m;
    pose_json["heading_deg"] = nullptr;
    pose_json["offsets"] = nullptr;
  } else {
    pose_json["lateral_offset_m"] = nullptr;
    pose_json["lateral_reference"] = nullptr;
    pose_json["lateral_reference_x_m"] = nullptr;
    pose_json["heading_deg"] = nullptr;
    pose_json["offsets"] = nullptr;
  }

  JsonObject can = document["can"].to<JsonObject>();
  can["enabled"] = false;
  can["state"] = "disabled";
  sendJson(document);
}

void handleConfig() {
  StaticJsonDocument<1024> document;
  document["activePairs"] = activePairsName();
  document["wheelbase"] = app_config.wheelbase_m;
  document["pivotSpacing"] = app_config.pivot_spacing_m;
  document["armLength"] = app_config.arm_length_m;
  document["contactLength"] = app_config.contact_length_m;
  document["frontOffset"] = app_config.pair_offset_m[kFront];
  document["rearOffset"] = app_config.pair_offset_m[kRear];
  document["frontRestAngle"] = app_config.rest_angle_deg[kFront];
  document["rearRestAngle"] = app_config.rest_angle_deg[kRear];
  document["trackWidth"] = app_config.track_width_m;
  document["contactThreshold"] = app_config.contact_threshold_deg;
  document["hysteresis"] = app_config.contact_hysteresis_deg;
  document["medianWindow"] = app_config.median_window;
  document["publishRate"] = app_config.publish_rate_hz;
  document["estimator"] = app_config.estimator == 2 ? "v2" : "v1";
  sendJson(document);
}

void handleCalibrationState() {
  DynamicJsonDocument document(2048);
  JsonObject sensors_json = document["sensors"].to<JsonObject>();
  for (int index = 0; index < kSensorChannelCount; ++index) {
    const SensorState& sensor = sensors[index];
    JsonObject item = sensors_json[kSensorNames[index]].to<JsonObject>();
    item["enabled"] = sensorEnabled(index);
    item["neutral"] = sensor.neutral_captured;
    item["neutral_v"] = sensor.calibration.voltage_neutral_v;
    item["direction"] = sensor.direction_captured;
    // Course disponible vers l'arrière avant la limite électrique du capteur :
    // un neutre trop près d'une extrémité limite la mesure.
    if (sensor.direction_captured) {
      const float headroom_v = sensor.calibration.invert
          ? sensor.calibration.voltage_neutral_v - 0.5f
          : 4.5f - sensor.calibration.voltage_neutral_v;
      item["backward_range_deg"] = headroom_v / kNominalVoltsPerDegree;
    } else {
      item["backward_range_deg"] = nullptr;
    }
    item["endpoints"] = sensor.endpoints_captured;
    item["min_v"] = sensor.calibration.voltage_min_v;
    item["max_v"] = sensor.calibration.voltage_max_v;
    item["calibrated"] = sensor.calibration.calibrated;
  }
  JsonObject center = document["center"].to<JsonObject>();
  center["front"] = center_captured[kFront];
  center["rear"] = center_captured[kRear];
  document["capturing_endpoints"] = endpoint_capture_active;
  document["capture_remaining_ms"] = endpoint_capture_active
      ? 10000 - std::min<std::uint32_t>(10000, millis() - endpoint_capture_started_ms)
      : 0;
  document["calibrated"] = calibrationComplete();
  sendJson(document);
}

int sensorFromCaptureName(const char* name) {
  static const char kPrefix[] = "neutral_";
  if (std::strncmp(name, kPrefix, sizeof(kPrefix) - 1) != 0) return -1;
  const char* sensor_name = name + sizeof(kPrefix) - 1;
  for (int index = 0; index < kSensorChannelCount; ++index) {
    if (std::strcmp(sensor_name, kSensorNames[index]) == 0) return index;
  }
  return -1;
}

void handleCalibrationCapture() {
  StaticJsonDocument<256> request;
  if (deserializeJson(request, web_server.arg("plain"))) {
    sendError(400, "JSON invalide");
    return;
  }
  const char* name = request["name"] | "";
  const int sensor_index = sensorFromCaptureName(name);
  if (sensor_index >= 0) {
    if (!sensorEnabled(sensor_index)) {
      sendError(409, "Capteur d'une paire désactivée");
      return;
    }
    if (voltage_history_count < kCalibrationWindowSize) {
      sendError(409, "Attendre deux secondes de mesures stables");
      return;
    }
    SensorState& sensor = sensors[sensor_index];
    float mean_voltage_v = 0.0f;
    if (!captureStableVoltage(sensor.history.data(), kCalibrationWindowSize,
                              0.12f, mean_voltage_v)) {
      sendError(409, "Mesure instable ou hors plage");
      return;
    }
    sensor.calibration.voltage_neutral_v = mean_voltage_v;
    sensor.neutral_captured = true;
    updateCalibrationValidity();
  } else if (std::strcmp(name, "direction") == 0) {
    // Tous les bras actifs sont tenus repoussés vers l'arrière : le signe de
    // l'écart au neutre donne le sens de chaque capteur (montage en miroir).
    constexpr std::size_t kDirectionSamples = kSampleRateHz / 10;  // 100 ms
    constexpr float kMinimumDeflectionV = 0.25f;  // ≈ 11°
    if (voltage_history_count < kDirectionSamples) {
      sendError(409, "Attendre quelques mesures");
      return;
    }
    float deltas[kSensorChannelCount] = {};
    for (int index = 0; index < kSensorChannelCount; ++index) {
      if (!sensorEnabled(index)) continue;
      const SensorState& sensor = sensors[index];
      if (!sensor.neutral_captured) {
        sendError(409, "Capturer d'abord le neutre de tous les bras actifs");
        return;
      }
      float sum = 0.0f;
      for (std::size_t i = 0; i < kDirectionSamples; ++i) {
        const std::size_t slot = (voltage_history_next + kCalibrationWindowSize - 1 - i) %
                                 kCalibrationWindowSize;
        sum += sensor.history[slot];
      }
      deltas[index] = sum / kDirectionSamples - sensor.calibration.voltage_neutral_v;
      if (!std::isfinite(deltas[index]) || std::fabs(deltas[index]) < kMinimumDeflectionV) {
        char message[96];
        std::snprintf(message, sizeof(message),
                      "Bras %s pas assez repoussé vers l'arrière", kSensorNames[index]);
        sendError(409, message);
        return;
      }
    }
    for (int index = 0; index < kSensorChannelCount; ++index) {
      if (!sensorEnabled(index)) continue;
      sensors[index].calibration.invert = deltas[index] < 0.0f;
      sensors[index].direction_captured = true;
    }
    updateCalibrationValidity();
  } else if (std::strcmp(name, "endpoints") == 0) {
    // Balayage simultané de tous les bras des paires actives pendant 10 s.
    endpoint_capture_active = true;
    endpoint_capture_started_ms = millis();
    for (int index = 0; index < kSensorChannelCount; ++index) {
      if (!sensorEnabled(index)) continue;
      sensors[index].endpoints_captured = false;
      sensors[index].capture_min_v = 4.7f;
      sensors[index].capture_max_v = 0.3f;
    }
    updateCalibrationValidity();
    persistCalibration();
    StaticJsonDocument<128> response;
    response["ok"] = true;
    response["capturing"] = true;
    response["duration_ms"] = 10000;
    sendJson(response, 202);
    return;
  } else if (std::strcmp(name, "center") == 0) {
    // Machine centrée et alignée dans un rang de référence : chaque paire
    // active doit être valide. Avec deux paires, cela remet aussi le cap à zéro.
    for (int pair = 0; pair < kPairCount; ++pair) {
      if (app_config.pair_enabled[pair] && !runtime_state.pairs[pair].valid) {
        sendError(409, pair == kFront ? "Paire avant invalide" : "Paire arrière invalide");
        return;
      }
    }
    for (int pair = 0; pair < kPairCount; ++pair) {
      if (!app_config.pair_enabled[pair]) continue;
      app_config.center_bias_m[pair] = runtime_state.raw_row_y_m[pair];
      if (std::isfinite(runtime_state.raw_lr_diff_deg[pair])) {
        app_config.center_lr_diff_deg[pair] = runtime_state.raw_lr_diff_deg[pair];
      }
      center_captured[pair] = true;
    }
    preferences.putFloat("center", app_config.center_bias_m[kFront]);
    preferences.putFloat("center_r", app_config.center_bias_m[kRear]);
    preferences.putFloat("cdiff", app_config.center_lr_diff_deg[kFront]);
    preferences.putFloat("cdiff_r", app_config.center_lr_diff_deg[kRear]);
  } else {
    sendError(400, "Étape inconnue");
    return;
  }
  persistCalibration();
  web_server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrationReset() {
  for (int index = 0; index < kSensorChannelCount; ++index) {
    sensors[index].calibration = SensorCalibration{};
    sensors[index].neutral_captured = false;
    sensors[index].endpoints_captured = false;
    sensors[index].direction_captured = false;
    preferences.remove(sensorKey(index, "min"));
    preferences.remove(sensorKey(index, "neu"));
    preferences.remove(sensorKey(index, "max"));
  }
  for (int pair = 0; pair < kPairCount; ++pair) {
    center_captured[pair] = false;
    app_config.center_bias_m[pair] = 0.0f;
    app_config.center_lr_diff_deg[pair] = 0.0f;
  }
  endpoint_capture_active = false;
  voltage_history_count = 0;
  preferences.putFloat("center", 0.0f);
  preferences.putFloat("center_r", 0.0f);
  preferences.putFloat("cdiff", 0.0f);
  preferences.putFloat("cdiff_r", 0.0f);
  updateCalibrationValidity();
  persistCalibration();
  web_server.send(200, "application/json", "{\"ok\":true}");
}

void handleConfigUpdate() {
  StaticJsonDocument<1536> request;
  if (deserializeJson(request, web_server.arg("plain"))) {
    sendError(400, "JSON invalide");
    return;
  }
  JsonObjectConst values = request["config"].as<JsonObjectConst>();
  if (values.isNull()) values = request.as<JsonObjectConst>();

  // On valide sur une copie : rien n'est appliqué si un seul champ est refusé.
  AppConfig next = app_config;
  float numeric = 0.0f;
  struct FloatField {
    const char* name;
    float minimum;
    float maximum;
    float* target;
  };
  const FloatField float_fields[] = {
      {"wheelbase", 2.0f, 6.0f, &next.wheelbase_m},
      {"armLength", 0.1f, 1.2f, &next.arm_length_m},
      {"pivotSpacing", 0.3f, 1.2f, &next.pivot_spacing_m},
      {"contactLength", 0.1f, 1.2f, &next.contact_length_m},
      {"frontOffset", -1.5f, 1.5f, &next.pair_offset_m[kFront]},
      {"rearOffset", -1.5f, 1.5f, &next.pair_offset_m[kRear]},
      {"trackWidth", 0.5f, 3.0f, &next.track_width_m},
      {"contactThreshold", 1.0f, 30.0f, &next.contact_threshold_deg},
      {"hysteresis", 0.0f, 10.0f, &next.contact_hysteresis_deg},
      {"frontRestAngle", 0.0f, 180.0f, &next.rest_angle_deg[kFront]},
      {"rearRestAngle", 0.0f, 180.0f, &next.rest_angle_deg[kRear]},
  };
  for (const FloatField& field : float_fields) {
    if (!values.containsKey(field.name)) continue;
    if (!validNumber(values[field.name], field.minimum, field.maximum, numeric)) {
      char message[64];
      std::snprintf(message, sizeof(message), "%s invalide", field.name);
      sendError(400, message);
      return;
    }
    *field.target = numeric;
  }
  if (values.containsKey("medianWindow")) {
    const int window = values["medianWindow"].as<int>();
    if (window < 3 || window > 21 || window % 2 == 0) return sendError(400, "medianWindow invalide");
    next.median_window = window;
  }
  if (values.containsKey("publishRate")) {
    const int rate = values["publishRate"].as<int>();
    if (rate < 5 || rate > 200) return sendError(400, "publishRate invalide");
    next.publish_rate_hz = rate;
  }
  if (values.containsKey("estimator")) {
    const char* estimator = values["estimator"] | "";
    if (std::strcmp(estimator, "v1") != 0 && std::strcmp(estimator, "v2") != 0) return sendError(400, "estimator invalide");
    next.estimator = std::strcmp(estimator, "v2") == 0 ? 2 : 1;
  }
  if (values.containsKey("activePairs")) {
    const char* active = values["activePairs"] | "";
    if (std::strcmp(active, "front") == 0) {
      next.pair_enabled[kFront] = true;
      next.pair_enabled[kRear] = false;
    } else if (std::strcmp(active, "rear") == 0) {
      next.pair_enabled[kFront] = false;
      next.pair_enabled[kRear] = true;
    } else if (std::strcmp(active, "both") == 0) {
      next.pair_enabled[kFront] = true;
      next.pair_enabled[kRear] = true;
    } else {
      return sendError(400, "activePairs invalide (front, rear ou both)");
    }
  }
  // Les deux stations doivent rester distinctes pour calculer un cap.
  if (next.wheelbase_m + next.pair_offset_m[kFront] - next.pair_offset_m[kRear] < 0.5f) {
    return sendError(400, "Paires avant et arrière trop proches");
  }

  app_config = next;
  preferences.putFloat("wheelbase", app_config.wheelbase_m);
  preferences.putFloat("arm_len", app_config.arm_length_m);
  preferences.putFloat("spacing", app_config.pivot_spacing_m);
  preferences.putFloat("contact", app_config.contact_length_m);
  preferences.putFloat("front_off", app_config.pair_offset_m[kFront]);
  preferences.putFloat("rear_off", app_config.pair_offset_m[kRear]);
  preferences.putFloat("rest_ang", app_config.rest_angle_deg[kFront]);
  preferences.putFloat("rest_ang_r", app_config.rest_angle_deg[kRear]);
  preferences.putBool("front_en", app_config.pair_enabled[kFront]);
  preferences.putBool("rear_en", app_config.pair_enabled[kRear]);
  preferences.putFloat("track", app_config.track_width_m);
  preferences.putFloat("threshold", app_config.contact_threshold_deg);
  preferences.putFloat("hyst", app_config.contact_hysteresis_deg);
  preferences.putInt("median", app_config.median_window);
  preferences.putInt("rate", app_config.publish_rate_hz);
  preferences.putInt("estimator", app_config.estimator);
  recreateChannels();
  StaticJsonDocument<128> response;
  response["ok"] = true;
  sendJson(response);
}

void handleOtaUpload() {
  HTTPUpload& upload = web_server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) ota_restart_at_ms = millis() + 1000;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void finishOtaUpload() {
  if (Update.hasError()) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"OTA échouée\"}");
  } else {
    web_server.send(200, "application/json", "{\"ok\":true,\"restarting\":true}");
  }
}

void configureRoutes() {
  web_server.on("/", HTTP_GET, []() {
    if (!LittleFS.exists("/index.html")) {
      web_server.send(503, "text/plain; charset=utf-8", "Page absente de LittleFS");
      return;
    }
    File page = LittleFS.open("/index.html", "r");
    web_server.streamFile(page, "text/html; charset=utf-8");
    page.close();
  });
  web_server.on("/api/health", HTTP_GET, []() {
    web_server.send(200, "application/json", "{\"ok\":true}");
  });
  web_server.on("/api/state", HTTP_GET, handleState);
  web_server.on("/api/config", HTTP_GET, handleConfig);
  web_server.on("/api/config", HTTP_POST, handleConfigUpdate);
  web_server.on("/api/calibration", HTTP_GET, handleCalibrationState);
  web_server.on("/api/calibration/capture", HTTP_POST, handleCalibrationCapture);
  web_server.on("/api/calibration/reset", HTTP_POST, handleCalibrationReset);
  web_server.on("/api/ota", HTTP_POST, finishOtaUpload, handleOtaUpload);
  web_server.onNotFound([]() {
    web_server.send(404, "application/json", "{\"ok\":false,\"error\":\"route inconnue\"}");
  });
}

}  // namespace

void setup() {
  Serial.begin(115200);
  preferences.begin("palpeur", false);
  loadConfig();
  beginBoardInputs();
  recreateChannels();
  LittleFS.begin(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(PALPEUR_WIFI_SSID, PALPEUR_WIFI_PASSWORD);
  MDNS.begin("palpeur");
  configureRoutes();
  web_server.begin();
  updateMeasurements();
  Serial.printf("Palpeur TRAXX: http://%s/ (paires: %s)\n",
                WiFi.softAPIP().toString().c_str(), activePairsName());
}

void loop() {
  web_server.handleClient();
  constexpr std::uint32_t sample_period_ms = 1000 / kSampleRateHz;
  if (millis() - last_sample_ms >= sample_period_ms) updateMeasurements();
  if (ota_restart_at_ms != 0 && static_cast<std::int32_t>(millis() - ota_restart_at_ms) >= 0) {
    ESP.restart();
  }
}

}  // namespace firmware
}  // namespace palpeur
