"""Test de fumée du serveur simulé : format JSON, passages de ceps, 1 / 2 paires."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import mock_server  # noqa: E402

failures = 0


def check(condition: bool, name: str) -> None:
    global failures
    print(f"[{'OK' if condition else 'FAIL'}] {name}")
    if not condition:
        failures += 1


def run(active: str, seconds: float = 6.0, step: float = 0.004) -> tuple[dict, int, int]:
    """Simule `seconds` de roulage à 6 km/h, horloge imposée (250 Hz)."""
    mock_server.STATE["config"]["activePairs"] = active
    mock_server.STATE["started_at"] = 0.0
    mock_server.STATE.pop("last_elapsed", None)
    for pair in mock_server.PAIRS:
        mock_server.TRACKERS[pair] = mock_server.new_tracker()
    valid_samples = samples = 0
    state: dict = {}
    t = 0.0
    while t < seconds:
        state = mock_server.simulated_state(now=t)
        json.dumps(state, allow_nan=False)  # lève une exception si NaN/Infinity
        samples += 1
        valid_samples += state["front_pair"]["valid"]
        t += step
    return state, valid_samples, samples


for active in ("front", "rear", "both"):
    state, valid_samples, samples = run(active)
    pose = state["pose"]
    check(state["active_pairs"] == active, f"{active}: active_pairs publié")
    check(state["front_pair"]["enabled"] == (active != "rear"), f"{active}: état paire avant")
    check(state["rear_pair"]["enabled"] == (active != "front"), f"{active}: état paire arrière")
    check(pose["lateral_valid"], f"{active}: décalage disponible à partir des derniers ceps")
    check(pose["heading_valid"] == (active == "both"), f"{active}: cap seulement avec 2 paires")
    if active == "both":
        check(pose["lateral_reference"] == "wheelbase_center", "both: référence centre d'empattement")
        check(abs(pose["heading_deg"]) < 1.5, "both: cap simulé plausible")
    else:
        check(pose["lateral_reference"] == f"{active}_pair", f"{active}: référence à la paire")
        check(pose["heading_deg"] is None, f"{active}: cap null, pas zéro")
    if active != "rear":
        last = state["front_pair"]["last_vine"]
        # 6 s à 6 km/h = 10 m : ~10 ceps et 1 à 2 piquets.
        check(last is not None and 8 <= last["count"] <= 14, "≈ un passage par cep ou piquet")
        check(0.2 < valid_samples / samples < 0.5, "double contact une partie du temps seulement")

left, right = mock_server.release_sweeps(0.05, 0.05)
check(left > right, "rang à gauche : bras gauche plus repoussé (G−D > 0)")
left, right = mock_server.release_sweeps(-0.05, 0.05)
check(left < right, "rang à droite : G−D < 0")
check(mock_server.MockHandler._validate_config({"frontRestAngle": 200}) is not None,
      "angle de repos hors 0–180° refusé")
check(mock_server.MockHandler._validate_config({"activePairs": "middle"}) is not None,
      "activePairs invalide refusé")
check(mock_server.MockHandler._validate_config({"activePairs": "both", "rearOffset": -0.4}) is None,
      "configuration 2 paires acceptée")

if failures:
    print(f"{failures} test(s) failed")
    sys.exit(1)
print("All mock tests passed")
