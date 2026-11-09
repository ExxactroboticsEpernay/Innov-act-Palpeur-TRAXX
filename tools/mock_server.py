#!/usr/bin/env python3
"""Serveur local de démonstration pour l'IHM du Palpeur TRAXX.

Le JSON produit suit le même format que le firmware (`/api/state`,
`/api/config`, `/api/calibration`), avec une ou deux paires de palpeurs.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = ROOT / "data"
PAIRS = ("front", "rear")
SENSORS = ("front_left", "front_right", "rear_left", "rear_right")
DEFAULT_CONFIG = {
    "activePairs": "front",
    "wheelbase": 2.14,
    "pivotSpacing": 0.515,
    "armLength": 0.395,
    "contactLength": 0.395,
    "frontOffset": 0.4,
    "rearOffset": -0.4,
    "frontArmDirection": "forward",
    "rearArmDirection": "forward",
    "trackWidth": 1.16,
    "estimator": "v1",
    "contactThreshold": 3.0,
    "hysteresis": 1.0,
    "medianWindow": 5,
    "publishRate": 50,
}
CONFIG_RANGES = {
    "wheelbase": (2.0, 6.0),
    "pivotSpacing": (0.3, 1.2),
    "armLength": (0.1, 1.2),
    "contactLength": (0.1, 1.2),
    "frontOffset": (-1.5, 1.5),
    "rearOffset": (-1.5, 1.5),
    "trackWidth": (0.5, 3.0),
    "contactThreshold": (1.0, 30.0),
    "hysteresis": (0.0, 10.0),
    "medianWindow": (3, 21),
    "publishRate": (5, 200),
}
CHOICES = {
    "activePairs": ("front", "rear", "both"),
    "frontArmDirection": ("forward", "backward"),
    "rearArmDirection": ("forward", "backward"),
    "estimator": ("v1", "v2"),
}
OBJECT_RADIUS_M = 0.05


def new_calibration() -> dict:
    return {
        "sensors": {name: {"neutral": False, "endpoints": False} for name in SENSORS},
        "center": {"front": False, "rear": False},
    }


STATE = {
    "started_at": time.monotonic(),
    "sequence": 0,
    "config": dict(DEFAULT_CONFIG),
    "calibration": new_calibration(),
}


def pair_enabled(pair: str) -> bool:
    active = STATE["config"]["activePairs"]
    return active == "both" or active == pair


def pair_x(pair: str) -> float:
    config = STATE["config"]
    if pair == "front":
        return config["wheelbase"] + config["frontOffset"]
    return config["rearOffset"]


def nominal_voltage(angle_deg: float) -> float:
    return 0.5 + (angle_deg + 90.0) * 4.0 / 180.0


def simulated_pair(pair: str, row_y_m: float, elapsed: float) -> dict:
    """Angles des bras touchant un objet rond centré en row_y_m (modèle v1)."""
    config = STATE["config"]
    disabled_sensor = {
        "voltage_v": None, "angle_raw_deg": None, "angle_filtered_deg": None,
        "contact": False, "fault": False,
    }
    if not pair_enabled(pair):
        return {
            "enabled": False, "valid": False, "reason": "pair_disabled",
            "x_m": round(pair_x(pair), 3), "row_y_m": None, "apparent_width_m": None,
            "left": dict(disabled_sensor), "right": dict(disabled_sensor),
        }

    half = config["pivotSpacing"] / 2.0
    reach = config["contactLength"]
    # Contact gauche au-dessus de l'objet, contact droit en dessous.
    sin_left = max(-1.0, min(1.0, (half - (row_y_m + OBJECT_RADIUS_M)) / reach))
    sin_right = max(-1.0, min(1.0, (half + (row_y_m - OBJECT_RADIUS_M)) / reach))
    jitter = 0.15 * math.sin(elapsed * 2.3 + (0.0 if pair == "front" else 1.7))
    angles = {
        "left": math.degrees(math.asin(sin_left)) + jitter,
        "right": math.degrees(math.asin(sin_right)) - jitter,
    }
    sides = {}
    for side, angle in angles.items():
        sides[side] = {
            "voltage_v": round(nominal_voltage(angle), 3),
            "angle_raw_deg": round(angle, 2),
            "angle_filtered_deg": round(angle - jitter * 0.8, 2),
            "contact": True,
            "fault": False,
        }
    return {
        "enabled": True, "valid": True, "reason": "none",
        "x_m": round(pair_x(pair), 3), "row_y_m": round(row_y_m, 5),
        "apparent_width_m": round(2 * OBJECT_RADIUS_M, 4),
        "left": sides["left"], "right": sides["right"],
    }


def simulated_state() -> dict:
    config = STATE["config"]
    elapsed = time.monotonic() - STATE["started_at"]
    sequence = STATE["sequence"]
    STATE["sequence"] = (sequence + 1) & 0xFFFFFFFF

    # Rang droit, machine qui serpente : décalage ±4 cm, lacet ±0,7°.
    front_x, rear_x = pair_x("front"), pair_x("rear")
    middle_x = (front_x + rear_x) / 2.0
    row_y_middle = 0.031 * math.sin(elapsed * 0.32) + 0.012 * math.sin(elapsed * 0.11)
    row_slope = math.tan(0.012 * math.sin(elapsed * 0.18))
    pairs = {
        pair: simulated_pair(pair, row_y_middle + row_slope * (pair_x(pair) - middle_x), elapsed)
        for pair in PAIRS
    }

    pose = {
        "lateral_valid": False, "heading_valid": False, "estimated": False,
        "reason": "none", "lateral_offset_m": None, "lateral_reference": None,
        "lateral_reference_x_m": None, "heading_deg": None, "offsets": None,
    }
    front, rear = pairs["front"], pairs["rear"]
    if front["valid"] and rear["valid"]:
        separation = front_x - rear_x

        def offset_at(x_ref: float) -> float:
            row_y = rear["row_y_m"] + (front["row_y_m"] - rear["row_y_m"]) * (x_ref - rear_x) / separation
            return round(-row_y, 5)

        wheelbase = config["wheelbase"]
        pose.update({
            "lateral_valid": True, "heading_valid": True,
            "lateral_offset_m": offset_at(wheelbase / 2.0),
            "lateral_reference": "wheelbase_center",
            "lateral_reference_x_m": wheelbase / 2.0,
            "heading_deg": round(-math.degrees(math.atan2(front["row_y_m"] - rear["row_y_m"], separation)), 3),
            "offsets": {
                "rear_axle_m": offset_at(0.0),
                "wheelbase_center_m": offset_at(wheelbase / 2.0),
                "front_axle_m": offset_at(wheelbase),
            },
        })
    else:
        valid_pair = "front" if front["valid"] else "rear" if rear["valid"] else None
        if valid_pair:
            pose.update({
                "lateral_valid": True,
                "lateral_offset_m": round(-pairs[valid_pair]["row_y_m"], 5),
                "lateral_reference": f"{valid_pair}_pair",
                "lateral_reference_x_m": pairs[valid_pair]["x_m"],
            })

    return {
        "timestamp_ms": int(elapsed * 1000),
        "sequence": sequence,
        "simulation": True,
        "active_pairs": config["activePairs"],
        "front_pair": front,
        "rear_pair": rear,
        "pose": pose,
        "can": {"enabled": False, "state": "disabled"},
    }


def calibration_state() -> dict:
    calibration = STATE["calibration"]
    sensors = {}
    for name in SENSORS:
        item = calibration["sensors"][name]
        enabled = pair_enabled(name.split("_")[0])
        sensors[name] = {
            "enabled": enabled,
            "neutral": item["neutral"],
            "neutral_v": 2.5,
            "endpoints": item["endpoints"],
            "min_v": 0.5,
            "max_v": 4.5,
            "calibrated": item["neutral"] and item["endpoints"],
        }
    complete = all(item["calibrated"] for item in sensors.values() if item["enabled"])
    return {
        "sensors": sensors,
        "center": dict(calibration["center"]),
        "capturing_endpoints": False,
        "capture_remaining_ms": 0,
        "calibrated": complete,
    }


class MockHandler(BaseHTTPRequestHandler):
    server_version = "PalpeurTRAXXMock/0.2"

    def log_message(self, format_string: str, *args: object) -> None:
        print(f"{self.log_date_time_string()} {format_string % args}")

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send_file(DATA_ROOT / "index.html", "text/html; charset=utf-8")
        elif path == "/api/health":
            self._send_json({"ok": True, "simulation": True, "version": "mock-0.2"})
        elif path == "/api/state":
            self._send_json(simulated_state())
        elif path == "/api/config":
            self._send_json(STATE["config"])
        elif path == "/api/calibration":
            self._send_json(calibration_state())
        else:
            self.send_error(404, "Route inconnue")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        payload = self._read_json()
        if payload is None:
            self.send_error(400, "JSON invalide")
            return

        if path == "/api/config":
            config = payload.get("config", payload)
            error = self._validate_config(config)
            if error:
                self._send_json({"ok": False, "error": error}, status=400)
                return
            STATE["config"].update({key: config[key] for key in config if key in DEFAULT_CONFIG})
            self._send_json({"ok": True, "config": STATE["config"]})
        elif path == "/api/calibration/capture":
            self._capture(str(payload.get("name", "")))
        elif path == "/api/calibration/reset":
            STATE["calibration"] = new_calibration()
            self._send_json({"ok": True})
        elif path == "/api/ota":
            self._send_json(
                {"ok": False, "error": "OTA indisponible sur le serveur simulé"},
                status=501,
            )
        else:
            self.send_error(404, "Route inconnue")

    def _capture(self, name: str) -> None:
        calibration = STATE["calibration"]
        if name.startswith("neutral_") and name[len("neutral_"):] in SENSORS:
            sensor = name[len("neutral_"):]
            if not pair_enabled(sensor.split("_")[0]):
                self._send_json({"ok": False, "error": "Capteur d'une paire désactivée"}, status=409)
                return
            calibration["sensors"][sensor]["neutral"] = True
        elif name == "endpoints":
            for sensor in SENSORS:
                if pair_enabled(sensor.split("_")[0]):
                    calibration["sensors"][sensor]["endpoints"] = True
        elif name == "center":
            for pair in PAIRS:
                if pair_enabled(pair):
                    calibration["center"][pair] = True
        else:
            self._send_json({"ok": False, "error": "Étape inconnue"}, status=400)
            return
        self._send_json({"ok": True})

    def _read_json(self) -> dict | None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 65536:
                return None
            payload = json.loads(self.rfile.read(length))
            return payload if isinstance(payload, dict) else None
        except (ValueError, json.JSONDecodeError):
            return None

    @staticmethod
    def _validate_config(config: object) -> str | None:
        if not isinstance(config, dict):
            return "La configuration doit être un objet JSON"
        for name, value in config.items():
            if name in CONFIG_RANGES:
                try:
                    numeric = float(value)
                except (TypeError, ValueError):
                    return f"{name} doit être numérique"
                minimum, maximum = CONFIG_RANGES[name]
                if not minimum <= numeric <= maximum:
                    return f"{name} doit être entre {minimum} et {maximum}"
            elif name in CHOICES and value not in CHOICES[name]:
                return f"{name} invalide"
        return None

    def _send_json(self, payload: object, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.is_file():
            self.send_error(404, "Fichier introuvable")
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser(description="IHM simulée du Palpeur TRAXX")
    parser.add_argument("--host", default="127.0.0.1", help="Adresse d'écoute locale")
    parser.add_argument("--port", type=int, default=8765, help="Port HTTP (défaut: 8765)")
    arguments = parser.parse_args()
    server = ThreadingHTTPServer((arguments.host, arguments.port), MockHandler)
    print(f"Maquette Palpeur TRAXX: http://{arguments.host}:{arguments.port}/")
    print("API simulée: /api/health, /api/state, /api/config, /api/calibration")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nArrêt du serveur simulé")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
