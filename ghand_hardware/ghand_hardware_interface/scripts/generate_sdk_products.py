#!/usr/bin/env python3
"""Generate ROS validation metadata from installed SDK product definitions."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--config-dir", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
rows = []
for product in ("ghand5", "ghandlite1"):
    config = json.loads((args.config_dir / f"{product}.json").read_text())
    joints = config["joints"]
    all_ids = ", ".join("JointId::" + joint["id"] for joint in joints)
    active_ids = ", ".join("JointId::" + joint["id"] for joint in joints
                           if "min" in joint and "max" in joint)
    rows.append('  {"' + product + '", {' + all_ids + '}, {' + active_ids + '}}')
args.output.write_text("// Generated from GHandSDK product JSON. Do not edit.\n"
                       "namespace ghand_comm {\ninline const ProductConfig sdk_products[] = {\n"
                       + ",\n".join(rows) + "\n};\n}\n")
