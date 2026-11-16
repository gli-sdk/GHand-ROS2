#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0
"""Prepare vendor Lite1 URDF paths and active limits without copying mesh assets."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    robot = ET.parse(args.source).getroot()
    robot.set("name", f"ghandlite1-{args.side}")
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        mesh.set("filename", filename.replace("ghand_lite1_system/meshes/",
                 "package://ghand_description/meshes/ghandlite1/"))
    for name, bounds in config["joint_limits_deg"].items():
        limit = robot.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f"Missing active joint limit in vendor URDF: {name}")
        limit.set("lower", str(math.radians(bounds["min"])))
        limit.set("upper", str(math.radians(bounds["max"])))
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
