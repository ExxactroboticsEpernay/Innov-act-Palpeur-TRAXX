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
    "frontRestAngle": 90.0,
    "rearRestAngle": 90.0,
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
    "frontRestAngle": (0.0, 180.0),
    "rearRestAngle": (0.0, 180.0),
}
CHOICES = {
    "activePairs": ("front", "rear", "both"),
    "estimator": ("v1", "v2"),
}
VOLTS_PER_DEGREE = 4.0 / 180.0
OBJECT_RADIUS_M = 0.05


def new_calibration() -> dict:
    return {
        "sensors": {
            name: {"neutral": False, "direction": False, "endpoints": False} for name in SENSORS
        },
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


# Rang réel : un cep tous les 1 m, un piquet tous les 7 m (entre deux ceps),
# TRAXX à 6 km/h. Chaque objet repousse les bras vers l'arrière pendant ~0,3 m
# de trajet, puis les libère (claquement amorti). Même modèle que la page.
SIM_SPEED_MPS = 6.0 / 3.6
VINE_SPACING_M = 1.0
STAKE_SPACING_M = 7.0
STAKE_OFFSET_M = 0.5
STAKE_DIAMETER_M = 0.06
VINE_HOLD_S = 5.0
MIN_EVENT_S = 0.03


def new_tracker() -> dict:
    return {"in_event": False, "sum_diff": 0.0, "sum_y": 0.0, "n": 0, "duration": 0.0, "last": None}


TRACKERS = {pair: new_tracker() for pair in PAIRS}


def update_tracker(tracker: dict, valid: bool, diff: float | None, row_y: float | None, dt: float) -> None:
    """Même logique que VineEventTracker (src/core/vine_events.h)."""
    last = tracker["last"]
    if last:
        last["age_s"] = round(last["age_s"] + dt, 3)
        last["fresh"] = last["age_s"] <= VINE_HOLD_S
    if valid and diff is not None and row_y is not None:
        if not tracker["in_event"]:
            tracker.update(in_event=True, sum_diff=0.0, sum_y=0.0, n=0, duration=0.0)
        tracker["sum_diff"] += diff
        tracker["sum_y"] += row_y
        tracker["n"] += 1
        tracker["duration"] += dt
        return
    if not tracker["in_event"]:
        return
    tracker["in_event"] = False
    if tracker["n"] == 0 or tracker["duration"] < MIN_EVENT_S:
        return
    tracker["last"] = {
        "lr_diff_deg": round(tracker["sum_diff"] / tracker["n"], 2),
        "row_y_m": round(tracker["sum_y"] / tracker["n"], 5),
        "duration_s": round(tracker["duration"], 3),
        "age_s": 0.0,
        "count": (last["count"] if last else 0) + 1,
        "fresh": True,
    }


def release_sweeps(row_y_m: float, radius_m: float) -> tuple[float, float]:
    """Balayages G/D (°, + vers l'arrière) quand l'objet quitte le bout des bras."""
    config = STATE["config"]
    half = config["pivotSpacing"] / 2.0
    reach = config["contactLength"]
    sin_left = max(-1.0, min(1.0, (half - (row_y_m + radius_m)) / reach))
    sin_right = max(-1.0, min(1.0, (half + (row_y_m - radius_m)) / reach))
    return 90.0 - math.degrees(math.asin(sin_left)), 90.0 - math.degrees(math.asin(sin_right))


def simulated_pair(pair: str, row_y_m: float, travel_m: float, dt: float) -> dict:
    """État instantané d'une paire et dernier passage de cep mesuré.

    angle_*_deg = balayage depuis le repos (+ vers l'arrière),
    arm_angle_deg = angle géométrique depuis l'axe x.
    """
    config = STATE["config"]
    disabled_sensor = {
        "voltage_v": None, "angle_raw_deg": None, "angle_filtered_deg": None,
        "arm_angle_deg": None, "contact": False, "fault": False, "rebound": False,
    }
    if not pair_enabled(pair):
        TRACKERS[pair] = new_tracker()
        return {
            "enabled": False, "valid": False, "reason": "pair_disabled",
            "x_m": round(pair_x(pair), 3), "row_y_m": None, "lr_diff_deg": None,
            "apparent_width_m": None, "last_vine": None,
            "left": dict(disabled_sensor), "right": dict(disabled_sensor),
        }

    # Dernier objet arrivé sur la ligne des bras au repos ; q = trajet depuis le contact.
    s = travel_m + pair_x(pair)
    q_vine = s - VINE_SPACING_M * math.floor(s / VINE_SPACING_M)
    stake_s = s - STAKE_OFFSET_M
    q_stake = stake_s - STAKE_SPACING_M * math.floor(stake_s / STAKE_SPACING_M)
    stake = q_stake < q_vine
    q = q_stake if stake else q_vine
    radius = STAKE_DIAMETER_M / 2.0 if stake else OBJECT_RADIUS_M

    release_left, release_right = release_sweeps(row_y_m, radius)
    mean_release = math.radians((release_left + release_right) / 2.0)
    contact_travel = config["armLength"] * math.sin(mean_release) + radius
    sweeps = {"left": 0.0, "right": 0.0}
    rebound = False
    if q <= contact_travel:
        sweeps = {"left": release_left * q / contact_travel, "right": release_right * q / contact_travel}
    else:
        tau = (q - contact_travel) / SIM_SPEED_MPS
        if tau < 0.15:
            snap = -6.0 * math.exp(-tau / 0.03) * math.cos(2.0 * math.pi * tau / 0.05)
            sweeps = {"left": snap, "right": snap}
            rebound = tau < 0.08

    rest = config[f"{pair}RestAngle"]
    threshold = config["contactThreshold"]
    # Capteurs montés en miroir : la tension monte à gauche et descend à droite.
    voltage_sign = {"left": 1.0, "right": -1.0}
    sides = {}
    for side, sweep in sweeps.items():
        sides[side] = {
            "voltage_v": round(2.5 + voltage_sign[side] * sweep * VOLTS_PER_DEGREE, 3),
            "angle_raw_deg": round(sweep, 2),
            "angle_filtered_deg": round(sweep, 2),
            "arm_angle_deg": round(rest + sweep, 2),
            "contact": (not rebound) and sweep > threshold,
            "fault": False,
            "rebound": rebound,
        }
    valid = sides["left"]["contact"] and sides["right"]["contact"]
    # Comparaison G/D (φG − φD) : > 0 = rang plus près du pivot gauche.
    diff = round(sweeps["left"] - sweeps["right"], 2) if valid else None
    update_tracker(TRACKERS[pair], valid, diff, row_y_m if valid else None, dt)
    if valid:
        reason = "none"
    elif sides["left"]["contact"]:
        reason = "right_free"
    elif sides["right"]["contact"]:
        reason = "left_free"
    else:
        reason = "both_free"
    last = TRACKERS[pair]["last"]
    return {
        "enabled": True, "valid": valid, "reason": reason,
        "x_m": round(pair_x(pair), 3), "row_y_m": round(row_y_m, 5) if valid else None,
        "lr_diff_deg": diff,
        "apparent_width_m": round(2 * radius, 4) if valid else None,
        "last_vine": dict(last) if last else None,
        "left": sides["left"], "right": sides["right"],
    }


def simulated_state(now: float | None = None) -> dict:
    """État simulé. `now` (s) permet aux tests d'imposer l'horloge."""
    config = STATE["config"]
    elapsed = (time.monotonic() if now is None else now) - STATE["started_at"]
    previous = STATE.get("last_elapsed")
    dt = 0.05 if previous is None else max(0.0, min(0.5, elapsed - previous))
    STATE["last_elapsed"] = elapsed
    sequence = STATE["sequence"]
    STATE["sequence"] = (sequence + 1) & 0xFFFFFFFF

    # Rang droit, machine qui serpente : décalage ±4 cm, lacet ±0,7°.
    travel = SIM_SPEED_MPS * elapsed
    front_x, rear_x = pair_x("front"), pair_x("rear")
    middle_x = (front_x + rear_x) / 2.0
    row_y_middle = 0.031 * math.sin(elapsed * 0.32) + 0.012 * math.sin(elapsed * 0.11)
    row_slope = math.tan(0.012 * math.sin(elapsed * 0.18))
    pairs = {
        pair: simulated_pair(pair, row_y_middle + row_slope * (pair_x(pair) - middle_x), travel, dt)
        for pair in PAIRS
    }

    # Pose à partir du dernier cep récent de chaque paire (comme le firmware).
    def held(pair: str) -> float | None:
        last = pairs[pair]["last_vine"]
        return last["row_y_m"] if pairs[pair]["enabled"] and last and last["fresh"] else None

    pose = {
        "lateral_valid": False, "heading_valid": False, "estimated": False,
        "reason": "no_recent_vine", "lateral_offset_m": None, "lateral_reference": None,
        "lateral_reference_x_m": None, "heading_deg": None, "offsets": None,
    }
    front_y, rear_y = held("front"), held("rear")
    if front_y is not None and rear_y is not None:
        separation = front_x - rear_x

        def offset_at(x_ref: float) -> float:
            return round(-(rear_y + (front_y - rear_y) * (x_ref - rear_x) / separation), 5)

        wheelbase = config["wheelbase"]
        pose.update({
            "lateral_valid": True, "heading_valid": True, "reason": "none",
            "lateral_offset_m": offset_at(wheelbase / 2.0),
            "lateral_reference": "wheelbase_center",
            "lateral_reference_x_m": wheelbase / 2.0,
            "heading_deg": round(-math.degrees(math.atan2(front_y - rear_y, separation)), 3),
            "offsets": {
                "rear_axle_m": offset_at(0.0),
                "wheelbase_center_m": offset_at(wheelbase / 2.0),
                "front_axle_m": offset_at(wheelbase),
            },
        })
    elif front_y is not None or rear_y is not None:
        valid_pair = "front" if front_y is not None else "rear"
        pose.update({
            "lateral_valid": True, "reason": "none",
            "lateral_offset_m": round(-(front_y if front_y is not None else rear_y), 5),
            "lateral_reference": f"{valid_pair}_pair",
            "lateral_reference_x_m": pairs[valid_pair]["x_m"],
        })

    return {
        "timestamp_ms": int(elapsed * 1000),
        "sequence": sequence,
        "simulation": True,
        "active_pairs": config["activePairs"],
        "front_pair": pairs["front"],
        "rear_pair": pairs["rear"],
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
            "direction": item["direction"],
            "backward_range_deg": 90.0 if item["direction"] else None,
            "endpoints": item["endpoints"],
            "min_v": 0.5,
            "max_v": 4.5,
            "calibrated": item["neutral"] and item["direction"] and item["endpoints"],
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
        elif name == "direction":
            active = [sensor for sensor in SENSORS if pair_enabled(sensor.split("_")[0])]
            if not all(calibration["sensors"][sensor]["neutral"] for sensor in active):
                self._send_json(
                    {"ok": False, "error": "Capturer d'abord le neutre de tous les bras actifs"},
                    status=409,
                )
                return
            for sensor in active:
                calibration["sensors"][sensor]["direction"] = True
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
