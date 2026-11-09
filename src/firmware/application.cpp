#include "application.h"

#include "board_io.h"
#include "core/calibration.h"
#include "core/geometry.h"
#include "core/sensor_pipeline.h"
#include "sim/geometry_simulation.h"

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

struct AppConfig {
  float wheelbase_m = 3.85f;
  float pivot_spacing_m = 0.525f;
  float arm_length_m = 0.35f;
  float contact_length_m = 0.35f;
  float front_pair_offset_m = 0.40f;
  float track_width_m = 1.16f;
  float center_bias_m = 0.0f;
  float contact_threshold_deg = 3.0f;
  float contact_hysteresis_deg = 1.0f;
  float smoothing_alpha = 0.25f;
  float maximum_slew_deg_per_second = 60.0f;
  float object_diameter_m = 0.10f;
  int arm_direction = 1;
  int median_window = 5;
  int publish_rate_hz = 50;
  int estimator = 1;
};

struct RuntimeState {
  SensorChannelReading left;
  SensorChannelReading right;
  PairEstimate pair;
  std::uint32_t sequence = 0;
  bool simulated = PALPEUR_SIMULATION != 0;
};

WebServer web_server(80);
Preferences preferences;
AppConfig app_config;
RuntimeState runtime_state;
SensorCalibration left_calibration;
SensorCalibration right_calibration;
SensorChannel* left_channel = nullptr;
SensorChannel* right_channel = nullptr;
std::uint32_t last_sample_ms = 0;
std::uint32_t ota_restart_at_ms = 0;
float simulation_phase = 0.0f;
constexpr std::size_t kCalibrationWindowSize = 100;
std::array<float, kCalibrationWindowSize> left_voltage_history{};
std::array<float, kCalibrationWindowSize> right_voltage_history{};
std::size_t voltage_history_next = 0;
std::size_t voltage_history_count = 0;
bool left_neutral_captured = false;
bool right_neutral_captured = false;
bool endpoints_captured = false;
bool center_captured = false;
bool endpoint_capture_active = false;
std::uint32_t endpoint_capture_started_ms = 0;
float capture_left_min_v = 4.7f;
float capture_left_max_v = 0.3f;
float capture_right_min_v = 4.7f;
float capture_right_max_v = 0.3f;

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
  delete left_channel;
  delete right_channel;
  left_channel = new SensorChannel(channelConfig(left_calibration));
  right_channel = new SensorChannel(channelConfig(right_calibration));
}

void loadConfig() {
  app_config.wheelbase_m = preferences.getFloat("wheelbase", 3.85f);
  app_config.pivot_spacing_m = preferences.getFloat("spacing", 0.525f);
  app_config.arm_length_m = preferences.getFloat("arm_len", 0.35f);
  app_config.contact_length_m = preferences.getFloat("contact", 0.35f);
  app_config.front_pair_offset_m = preferences.getFloat("front_off", 0.40f);
  app_config.track_width_m = preferences.getFloat("track", 1.16f);
  app_config.center_bias_m = preferences.getFloat("center", 0.0f);
  app_config.contact_threshold_deg = preferences.getFloat("threshold", 3.0f);
  app_config.contact_hysteresis_deg = preferences.getFloat("hyst", 1.0f);
  app_config.smoothing_alpha = preferences.getFloat("smooth", 0.25f);
  app_config.maximum_slew_deg_per_second = preferences.getFloat("slew", 60.0f);
  app_config.object_diameter_m = preferences.getFloat("diameter", 0.10f);
  app_config.arm_direction = preferences.getInt("arm_dir", 1);
  app_config.median_window = preferences.getInt("median", 5);
  app_config.publish_rate_hz = preferences.getInt("rate", 50);
  app_config.estimator = preferences.getInt("estimator", 1);

  left_calibration.voltage_min_v = preferences.getFloat("l_min", 0.5f);
  left_calibration.voltage_neutral_v = preferences.getFloat("l_neutral", 2.5f);
  left_calibration.voltage_max_v = preferences.getFloat("l_max", 4.5f);
  left_calibration.invert = preferences.getBool("l_invert", false);
  left_neutral_captured = preferences.getBool("ln_set", false);
  right_calibration.voltage_min_v = preferences.getFloat("r_min", 0.5f);
  right_calibration.voltage_neutral_v = preferences.getFloat("r_neutral", 2.5f);
  right_calibration.voltage_max_v = preferences.getFloat("r_max", 4.5f);
  right_calibration.invert = preferences.getBool("r_invert", false);
  right_neutral_captured = preferences.getBool("rn_set", false);
  endpoints_captured = preferences.getBool("endpoints", false);
  center_captured = preferences.getBool("center_set", false);

#if PALPEUR_SIMULATION
  left_calibration.calibrated = true;
  right_calibration.calibrated = true;
#else
  left_calibration.calibrated = preferences.getBool("l_valid", false);
  right_calibration.calibrated = preferences.getBool("r_valid", false);
#endif
}

bool validNumber(JsonVariantConst value, float minimum, float maximum,
                 float& output) {
  if (!value.is<float>() && !value.is<int>()) return false;
  output = value.as<float>();
  return std::isfinite(output) && output >= minimum && output <= maximum;
}

bool updateCalibrationValidity() {
  SensorCalibration left_check = left_calibration;
  SensorCalibration right_check = right_calibration;
  left_check.calibrated = true;
  right_check.calibrated = true;
  left_calibration.calibrated = left_neutral_captured && endpoints_captured &&
                                isCalibrationValid(left_check);
  right_calibration.calibrated = right_neutral_captured && endpoints_captured &&
                                 isCalibrationValid(right_check);
  preferences.putBool("l_valid", left_calibration.calibrated);
  preferences.putBool("r_valid", right_calibration.calibrated);
  recreateChannels();
  return left_calibration.calibrated && right_calibration.calibrated;
}

void persistCalibration() {
  preferences.putFloat("l_min", left_calibration.voltage_min_v);
  preferences.putFloat("l_neutral", left_calibration.voltage_neutral_v);
  preferences.putFloat("l_max", left_calibration.voltage_max_v);
  preferences.putFloat("r_min", right_calibration.voltage_min_v);
  preferences.putFloat("r_neutral", right_calibration.voltage_neutral_v);
  preferences.putFloat("r_max", right_calibration.voltage_max_v);
  preferences.putBool("ln_set", left_neutral_captured);
  preferences.putBool("rn_set", right_neutral_captured);
  preferences.putBool("endpoints", endpoints_captured);
  preferences.putBool("center_set", center_captured);
}

void finishEndpointCapture() {
  endpoint_capture_active = false;
  const bool left_span_valid = capture_left_max_v - capture_left_min_v >= 2.0f;
  const bool right_span_valid = capture_right_max_v - capture_right_min_v >= 2.0f;
  endpoints_captured = left_span_valid && right_span_valid;
  if (endpoints_captured) {
    left_calibration.voltage_min_v = capture_left_min_v;
    left_calibration.voltage_max_v = capture_left_max_v;
    right_calibration.voltage_min_v = capture_right_min_v;
    right_calibration.voltage_max_v = capture_right_max_v;
  }
  updateCalibrationValidity();
  persistCalibration();
}

void recordCalibrationSample(float left_voltage_v, float right_voltage_v,
                             std::uint32_t now) {
  left_voltage_history[voltage_history_next] = left_voltage_v;
  right_voltage_history[voltage_history_next] = right_voltage_v;
  voltage_history_next = (voltage_history_next + 1) % kCalibrationWindowSize;
  if (voltage_history_count < kCalibrationWindowSize) ++voltage_history_count;

  if (!endpoint_capture_active) return;
  if (left_voltage_v >= 0.3f && left_voltage_v <= 4.7f) {
    capture_left_min_v = std::min(capture_left_min_v, left_voltage_v);
    capture_left_max_v = std::max(capture_left_max_v, left_voltage_v);
  }
  if (right_voltage_v >= 0.3f && right_voltage_v <= 4.7f) {
    capture_right_min_v = std::min(capture_right_min_v, right_voltage_v);
    capture_right_max_v = std::max(capture_right_max_v, right_voltage_v);
  }
  if (now - endpoint_capture_started_ms >= 10000) finishEndpointCapture();
}

void sendJson(JsonDocument& document, int status = 200) {
  String body;
  serializeJson(document, body);
  web_server.send(status, "application/json; charset=utf-8", body);
}

void updateMeasurements() {
  const std::uint32_t now = millis();
  const float delta_seconds = last_sample_ms == 0
      ? 0.02f
      : static_cast<float>(now - last_sample_ms) / 1000.0f;
  last_sample_ms = now;
  PairConfig pair_config;
  pair_config.x_m = app_config.wheelbase_m + app_config.front_pair_offset_m;
  pair_config.pivot_spacing_m = app_config.pivot_spacing_m;
  pair_config.contact_length_m = app_config.contact_length_m;
  pair_config.arm_direction = app_config.arm_direction;

  float left_voltage = 0.0f;
  float right_voltage = 0.0f;
#if PALPEUR_SIMULATION
  simulation_phase += delta_seconds;
  CircularObjectScenario scenario;
  scenario.pair = pair_config;
  scenario.diameter_m = app_config.object_diameter_m;
  scenario.row_y_at_pair_m = 0.035f * std::sin(simulation_phase * 0.32f);
  scenario.row_slope = std::tan(0.035f * std::sin(simulation_phase * 0.18f));
  const SimulatedContact simulated = simulateCircularObjectContact(scenario);
  if (simulated.valid) {
    left_voltage = 0.5f + (simulated.input.left_angle_deg + 90.0f) * 4.0f / 180.0f;
    right_voltage = 0.5f + (simulated.input.right_angle_deg + 90.0f) * 4.0f / 180.0f;
  } else {
    left_voltage = NAN;
    right_voltage = NAN;
  }
#else
  left_voltage = readLeftSensorVoltage();
  right_voltage = readRightSensorVoltage();
#endif

  recordCalibrationSample(left_voltage, right_voltage, now);

  runtime_state.left = left_channel->update(left_voltage, delta_seconds);
  runtime_state.right = right_channel->update(right_voltage, delta_seconds);
  PairInput pair_input;
  pair_input.left_angle_deg = runtime_state.left.filtered_angle_deg;
  pair_input.right_angle_deg = runtime_state.right.filtered_angle_deg;
  pair_input.left_contact = runtime_state.left.contact;
  pair_input.right_contact = runtime_state.right.contact;
  pair_input.left_sensor_valid = runtime_state.left.valid;
  pair_input.right_sensor_valid = runtime_state.right.valid;
  pair_input.calibration_valid = left_calibration.calibrated &&
                                 right_calibration.calibrated;

  if (app_config.estimator == 2) {
    runtime_state.pair = BisectorEstimator().estimate(pair_config, pair_input);
  } else {
    runtime_state.pair = ContactPointEstimator().estimate(pair_config, pair_input);
  }
  ++runtime_state.sequence;
}

void handleState() {
  StaticJsonDocument<1024> document;
  document["timestamp_ms"] = millis();
  document["sequence"] = runtime_state.sequence;
  document["simulation"] = runtime_state.simulated;
  JsonObject front = document["front_pair"].to<JsonObject>();
  front["enabled"] = true;
  front["valid"] = runtime_state.pair.valid;
  front["reason"] = runtime_state.pair.valid ? "none" : "invalid";
  JsonObject left = front["left"].to<JsonObject>();
  left["voltage_v"] = runtime_state.left.voltage_v;
  left["angle_raw_deg"] = runtime_state.left.raw_angle_deg;
  left["angle_filtered_deg"] = runtime_state.left.filtered_angle_deg;
  left["contact"] = runtime_state.left.contact;
  left["fault"] = !runtime_state.left.valid;
  JsonObject right = front["right"].to<JsonObject>();
  right["voltage_v"] = runtime_state.right.voltage_v;
  right["angle_raw_deg"] = runtime_state.right.raw_angle_deg;
  right["angle_filtered_deg"] = runtime_state.right.filtered_angle_deg;
  right["contact"] = runtime_state.right.contact;
  right["fault"] = !runtime_state.right.valid;
  JsonObject pose = document["pose"].to<JsonObject>();
  pose["lateral_offset_m"] = runtime_state.pair.valid
      ? -runtime_state.pair.row_y_m + app_config.center_bias_m
      : 0.0f;
  pose["lateral_valid"] = runtime_state.pair.valid;
  pose["heading_deg"] = nullptr;
  pose["heading_valid"] = false;
  pose["estimated"] = runtime_state.pair.estimated;
  JsonObject can = document["can"].to<JsonObject>();
  can["enabled"] = false;
  can["state"] = "disabled";
  sendJson(document);
}

void handleConfig() {
  StaticJsonDocument<1024> document;
  document["wheelbase"] = app_config.wheelbase_m;
  document["pivotSpacing"] = app_config.pivot_spacing_m;
  document["armLength"] = app_config.arm_length_m;
  document["contactLength"] = app_config.contact_length_m;
  document["frontOffset"] = app_config.front_pair_offset_m;
  document["trackWidth"] = app_config.track_width_m;
  document["contactThreshold"] = app_config.contact_threshold_deg;
  document["hysteresis"] = app_config.contact_hysteresis_deg;
  document["medianWindow"] = app_config.median_window;
  document["publishRate"] = app_config.publish_rate_hz;
  document["armDirection"] = app_config.arm_direction > 0 ? "forward" : "backward";
  document["estimator"] = app_config.estimator == 2 ? "v2" : "v1";
  sendJson(document);
}

void handleCalibrationState() {
  StaticJsonDocument<512> document;
  document["neutral_left"] = left_neutral_captured;
  document["neutral_right"] = right_neutral_captured;
  document["endpoints"] = endpoints_captured;
  document["center"] = center_captured;
  document["capturing_endpoints"] = endpoint_capture_active;
  document["capture_remaining_ms"] = endpoint_capture_active
      ? 10000 - std::min<std::uint32_t>(10000, millis() - endpoint_capture_started_ms)
      : 0;
  document["calibrated"] = left_neutral_captured && right_neutral_captured &&
                            endpoints_captured && left_calibration.calibrated &&
                            right_calibration.calibrated;
  document["left_neutral_v"] = left_calibration.voltage_neutral_v;
  document["right_neutral_v"] = right_calibration.voltage_neutral_v;
  sendJson(document);
}

void handleCalibrationCapture() {
  StaticJsonDocument<256> request;
  if (deserializeJson(request, web_server.arg("plain"))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"JSON invalide\"}");
    return;
  }
  const char* name = request["name"] | "";
  if (strcmp(name, "neutral_left") == 0 || strcmp(name, "neutral_right") == 0) {
    if (voltage_history_count < kCalibrationWindowSize) {
      web_server.send(409, "application/json", "{\"ok\":false,\"error\":\"Attendre deux secondes de mesures stables\"}");
      return;
    }
    float mean_voltage_v = 0.0f;
    const float* samples = strcmp(name, "neutral_left") == 0
        ? left_voltage_history.data()
        : right_voltage_history.data();
    if (!captureStableVoltage(samples, kCalibrationWindowSize, 0.12f,
                              mean_voltage_v)) {
      web_server.send(409, "application/json", "{\"ok\":false,\"error\":\"Mesure instable ou hors plage\"}");
      return;
    }
    if (strcmp(name, "neutral_left") == 0) {
      left_calibration.voltage_neutral_v = mean_voltage_v;
      left_neutral_captured = true;
    } else {
      right_calibration.voltage_neutral_v = mean_voltage_v;
      right_neutral_captured = true;
    }
    updateCalibrationValidity();
    persistCalibration();
  } else if (strcmp(name, "endpoints") == 0) {
    endpoint_capture_active = true;
    endpoints_captured = false;
    endpoint_capture_started_ms = millis();
    capture_left_min_v = 4.7f;
    capture_left_max_v = 0.3f;
    capture_right_min_v = 4.7f;
    capture_right_max_v = 0.3f;
    updateCalibrationValidity();
    persistCalibration();
    web_server.send(202, "application/json", "{\"ok\":true,\"capturing\":true,\"duration_ms\":10000}");
    return;
  } else if (strcmp(name, "center") == 0) {
    if (!runtime_state.pair.valid) {
      web_server.send(409, "application/json", "{\"ok\":false,\"error\":\"Paire invalide\"}");
      return;
    }
    app_config.center_bias_m = runtime_state.pair.row_y_m;
    center_captured = true;
    preferences.putFloat("center", app_config.center_bias_m);
    preferences.putBool("center_set", true);
  } else {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Étape inconnue\"}");
    return;
  }
  persistCalibration();
  recreateChannels();
  web_server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrationReset() {
  left_calibration = SensorCalibration{};
  right_calibration = SensorCalibration{};
  left_neutral_captured = false;
  right_neutral_captured = false;
  endpoints_captured = false;
  center_captured = false;
  endpoint_capture_active = false;
  voltage_history_count = 0;
  app_config.center_bias_m = 0.0f;
  preferences.remove("l_min");
  preferences.remove("l_neutral");
  preferences.remove("l_max");
  preferences.remove("r_min");
  preferences.remove("r_neutral");
  preferences.remove("r_max");
  preferences.putBool("l_valid", false);
  preferences.putBool("r_valid", false);
  preferences.putBool("ln_set", false);
  preferences.putBool("rn_set", false);
  preferences.putBool("endpoints", false);
  preferences.putBool("center_set", false);
  preferences.putFloat("center", 0.0f);
#if PALPEUR_SIMULATION
  left_calibration.calibrated = true;
  right_calibration.calibrated = true;
#endif
  recreateChannels();
  web_server.send(200, "application/json", "{\"ok\":true}");
}

void handleConfigUpdate() {
  StaticJsonDocument<1024> request;
  if (deserializeJson(request, web_server.arg("plain"))) {
    StaticJsonDocument<128> response;
    response["ok"] = false;
    response["error"] = "JSON invalide";
    sendJson(response, 400);
    return;
  }
  JsonObjectConst values = request["config"].as<JsonObjectConst>();
  if (values.isNull()) values = request.as<JsonObjectConst>();
  float numeric = 0.0f;
  if (values.containsKey("armLength")) {
    if (!validNumber(values["armLength"], 0.1f, 1.2f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"armLength invalide\"}");
    app_config.arm_length_m = numeric;
  }
  if (values.containsKey("pivotSpacing")) {
    if (!validNumber(values["pivotSpacing"], 0.3f, 1.2f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"pivotSpacing invalide\"}");
    app_config.pivot_spacing_m = numeric;
  }
  if (values.containsKey("contactLength")) {
    if (!validNumber(values["contactLength"], 0.1f, 1.2f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"contactLength invalide\"}");
    app_config.contact_length_m = numeric;
  }
  if (values.containsKey("frontOffset")) {
    if (!validNumber(values["frontOffset"], -1.5f, 1.5f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"frontOffset invalide\"}");
    app_config.front_pair_offset_m = numeric;
  }
  if (values.containsKey("trackWidth")) {
    if (!validNumber(values["trackWidth"], 0.5f, 3.0f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"trackWidth invalide\"}");
    app_config.track_width_m = numeric;
  }
  if (values.containsKey("contactThreshold")) {
    if (!validNumber(values["contactThreshold"], 1.0f, 30.0f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"contactThreshold invalide\"}");
    app_config.contact_threshold_deg = numeric;
  }
  if (values.containsKey("hysteresis")) {
    if (!validNumber(values["hysteresis"], 0.0f, 10.0f, numeric)) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"hysteresis invalide\"}");
    app_config.contact_hysteresis_deg = numeric;
  }
  if (values.containsKey("medianWindow")) {
    const int window = values["medianWindow"].as<int>();
    if (window < 3 || window > 21 || window % 2 == 0) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"medianWindow invalide\"}");
    app_config.median_window = window;
  }
  if (values.containsKey("publishRate")) {
    const int rate = values["publishRate"].as<int>();
    if (rate < 5 || rate > 200) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"publishRate invalide\"}");
    app_config.publish_rate_hz = rate;
  }
  if (values.containsKey("armDirection")) {
    const char* direction = values["armDirection"] | "forward";
    if (strcmp(direction, "forward") != 0 && strcmp(direction, "backward") != 0) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"armDirection invalide\"}");
    app_config.arm_direction = strcmp(direction, "forward") == 0 ? 1 : -1;
  }
  if (values.containsKey("estimator")) {
    const char* estimator = values["estimator"] | "v1";
    if (strcmp(estimator, "v1") != 0 && strcmp(estimator, "v2") != 0) return web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"estimator invalide\"}");
    app_config.estimator = strcmp(estimator, "v2") == 0 ? 2 : 1;
  }

  preferences.putFloat("arm_len", app_config.arm_length_m);
  preferences.putFloat("spacing", app_config.pivot_spacing_m);
  preferences.putFloat("contact", app_config.contact_length_m);
  preferences.putFloat("front_off", app_config.front_pair_offset_m);
  preferences.putFloat("track", app_config.track_width_m);
  preferences.putFloat("threshold", app_config.contact_threshold_deg);
  preferences.putFloat("hyst", app_config.contact_hysteresis_deg);
  preferences.putInt("median", app_config.median_window);
  preferences.putInt("rate", app_config.publish_rate_hz);
  preferences.putInt("arm_dir", app_config.arm_direction);
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
  Serial.printf("Palpeur TRAXX: http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  web_server.handleClient();
  const std::uint32_t sample_period_ms =
      std::max<std::uint32_t>(1, 1000 / app_config.publish_rate_hz);
  if (millis() - last_sample_ms >= sample_period_ms) updateMeasurements();
  if (ota_restart_at_ms != 0 && static_cast<std::int32_t>(millis() - ota_restart_at_ms) >= 0) {
    ESP.restart();
  }
}

}  // namespace firmware
}  // namespace palpeur