#!/usr/bin/env python3
"""Serveur local de démonstration pour l'IHM du Palpeur TRAXX."""

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
DEFAULT_CONFIG = {
    "pivotSpacing": 0.525,
    "armLength": 0.35,
    "contactLength": 0.35,
    "armDirection": "forward",
    "frontOffset": 0.4,
    "trackWidth": 1.16,
    "estimator": "v1",
    "contactThreshold": 3.0,
    "hysteresis": 1.0,
    "medianWindow": 5,
    "publishRate": 50,
}
CONFIG_RANGES = {
    "pivotSpacing": (0.3, 1.2),
    "armLength": (0.1, 1.2),
    "contactLength": (0.1, 1.2),
    "frontOffset": (-1.5, 1.5),
    "trackWidth": (0.5, 3.0),
    "contactThreshold": (1.0, 30.0),
    "hysteresis": (0.0, 10.0),
    "medianWindow": (3, 21),
    "publishRate": (5, 200),
}
STATE = {
    "started_at": time.monotonic(),
    "sequence": 0,
    "config": dict(DEFAULT_CONFIG),
    "calibration": {
        "neutral_left": False,
        "neutral_right": False,
        "endpoints": False,
        "center": False,
    },
}


def simulated_state() -> dict:
    elapsed = time.monotonic() - STATE["started_at"]
    sequence = STATE["sequence"]
    STATE["sequence"] = (sequence + 1) & 0xFFFFFFFF

    left_angle = 23.8 + 1.8 * math.sin(elapsed * 0.55)
    right_angle = -23.5 - 1.5 * math.sin(elapsed * 0.49)
    left_voltage = 2.94 + 0.04 * math.sin(elapsed * 0.8)
    right_voltage = 2.06 + 0.035 * math.sin(elapsed * 0.72)
    offset_m = (3.1 * math.sin(elapsed * 0.32) +
                1.2 * math.sin(elapsed * 0.11)) / 100.0

    return {
        "timestamp_ms": int(elapsed * 1000),
        "sequence": sequence,
        "simulation": True,
        "front_pair": {
            "enabled": True,
            "valid": True,
            "reason": "none",
            "left": {
                "voltage_v": round(left_voltage, 3),
                "angle_raw_deg": round(left_angle, 2),
                "angle_filtered_deg": round(left_angle - 0.2, 2),
                "contact": True,
                "fault": False,
            },
            "right": {
                "voltage_v": round(right_voltage, 3),
                "angle_raw_deg": round(right_angle, 2),
                "angle_filtered_deg": round(right_angle + 0.3, 2),
                "contact": True,
                "fault": False,
            },
        },
        "pose": {
            "lateral_offset_m": round(offset_m, 5),
            "lateral_valid": True,
            "heading_deg": None,
            "heading_valid": False,
            "estimated": False,
        },
        "can": {"enabled": False, "state": "disabled"},
    }


class MockHandler(BaseHTTPRequestHandler):
    server_version = "PalpeurTRAXXMock/0.1"

    def log_message(self, format_string: str, *args: object) -> None:
        print(f"{self.log_date_time_string()} {format_string % args}")

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send_file(DATA_ROOT / "index.html", "text/html; charset=utf-8")
        elif path == "/api/health":
            self._send_json({"ok": True, "simulation": True, "version": "mock-0.1"})
        elif path == "/api/state":
            self._send_json(simulated_state())
        elif path == "/api/config":
            self._send_json(STATE["config"])
        elif path == "/api/calibration":
            calibration = dict(STATE["calibration"])
            calibration["capturing_endpoints"] = False
            calibration["capture_remaining_ms"] = 0
            calibration["calibrated"] = all(
                STATE["calibration"][name]
                for name in ("neutral_left", "neutral_right", "endpoints")
            )
            calibration["left_neutral_v"] = 2.5
            calibration["right_neutral_v"] = 2.5
            self._send_json(calibration)
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
            STATE["config"].update(config)
            self._send_json({"ok": True, "config": STATE["config"]})
        elif path == "/api/calibration/capture":
            name = payload.get("name")
            if name not in STATE["calibration"]:
                self._send_json({"ok": False, "error": "Étape inconnue"}, status=400)
                return
            STATE["calibration"][name] = True
            self._send_json({"ok": True, "calibration": STATE["calibration"]})
        elif path == "/api/calibration/reset":
            for name in STATE["calibration"]:
                STATE["calibration"][name] = False
            self._send_json({"ok": True, "calibration": STATE["calibration"]})
        elif path == "/api/ota":
            self._send_json(
                {"ok": False, "error": "OTA indisponible sur le serveur simulé"},
                status=501,
            )
        else:
            self.send_error(404, "Route inconnue")

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
            elif name == "armDirection" and value not in ("forward", "backward"):
                return "armDirection invalide"
            elif name == "estimator" and value not in ("v1", "v2"):
                return "estimator invalide"
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