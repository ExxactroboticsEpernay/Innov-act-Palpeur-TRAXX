"""Test de fumée du serveur simulé : format JSON et logique 1 / 2 paires."""

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


for active in ("front", "rear", "both"):
    mock_server.STATE["config"]["activePairs"] = active
    state = mock_server.simulated_state()
    json.dumps(state, allow_nan=False)  # lève une exception si NaN/Infinity
    pose = state["pose"]
    check(state["active_pairs"] == active, f"{active}: active_pairs publié")
    check(state["front_pair"]["enabled"] == (active != "rear"), f"{active}: état paire avant")
    check(state["rear_pair"]["enabled"] == (active != "front"), f"{active}: état paire arrière")
    check(pose["lateral_valid"], f"{active}: décalage disponible")
    check(pose["heading_valid"] == (active == "both"), f"{active}: cap seulement avec 2 paires")
    if active == "both":
        check(pose["lateral_reference"] == "wheelbase_center", "both: référence centre d'empattement")
        check(abs(pose["heading_deg"]) < 1.0, "both: cap simulé plausible (< 1°)")
        check(set(pose["offsets"]) == {"rear_axle_m", "wheelbase_center_m", "front_axle_m"},
              "both: décalages aux trois références")
    else:
        check(pose["lateral_reference"] == f"{active}_pair", f"{active}: référence à la paire")
        check(pose["heading_deg"] is None, f"{active}: cap null, pas zéro")

check(mock_server.MockHandler._validate_config({"activePairs": "middle"}) is not None,
      "activePairs invalide refusé")
check(mock_server.MockHandler._validate_config({"activePairs": "both", "rearOffset": -0.4}) is None,
      "configuration 2 paires acceptée")

if failures:
    print(f"{failures} test(s) failed")
    sys.exit(1)
print("All mock tests passed")
