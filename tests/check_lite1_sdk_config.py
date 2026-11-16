"""Compare the local Lite1 product facts with a selected SDK checkout (read only)."""

import argparse
import json
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-dir", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    facts = yaml.safe_load((root / "ghand_description/config/ghandlite1.yaml").read_text(encoding="utf-8"))
    sdk = json.loads((args.sdk_dir / "config/ghandlite1.json").read_text(encoding="utf-8"))
    aliases = {f"{finger}_PIP": f"{finger}_IP" for finger in ("FF", "MF", "RF", "LF")}
    joints = [aliases.get(joint["id"], joint["id"]) for joint in sdk["joints"]]
    limits = {joint["id"]: {key: joint[key] for key in ("min", "max")}
              for joint in sdk["joints"] if "min" in joint}
    expected = {
        "joints": joints,
        "active_joints": [name for name in joints if name in limits],
        "joint_limits_deg": limits,
        "tactile_region_counts": [region["count"] for region in sdk["tactile_regions"]],
    }
    differences = [name for name, value in expected.items() if facts[name] != value]
    if differences:
        raise SystemExit("Lite1 SDK configuration differs: " + ", ".join(differences))
    print("Lite1 joint set, active joints, degree limits and tactile layout match the selected SDK")


if __name__ == "__main__":
    main()
