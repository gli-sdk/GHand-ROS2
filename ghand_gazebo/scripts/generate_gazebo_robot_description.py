#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Generate robot_description for Gazebo with patched joint lower limits.

This script calls xacro on the provided URDF/xacro file and then patches every
joint limit whose lower bound is exactly 0 to a small negative tolerance. This
works around Gazebo's physics solver reporting joint positions slightly below
zero (around -1e-6), which would otherwise violate the URDF lower limit.
"""

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


def patch_lower_limits(
    urdf_str: str, epsilon: float = -0.01, *, ghand5_dynamics: bool = False
) -> tuple[str, int]:
    """Patch zero lower joint limits to the given negative tolerance."""
    root = ET.fromstring(urdf_str)
    patched = 0
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        if ghand5_dynamics:
            for interface in ("effort", "velocity"):
                if float(limit.get(interface, "0")) == 0.0:
                    limit.set(interface, "50")
            if joint.find("dynamics") is None:
                ET.SubElement(joint, "dynamics", damping="0.1", friction="0.05")
        lower = limit.get("lower")
        if lower is None:
            continue
        try:
            if float(lower) == 0.0:
                # Format without trailing zeros to keep the URDF tidy.
                formatted = f"{epsilon:.6f}".rstrip("0").rstrip(".")
                limit.set("lower", formatted)
                patched += 1
        except ValueError:
            continue
    return ET.tostring(root, encoding="unicode"), patched


def add_mimic_joints(urdf_str: str, mimic_map: dict) -> tuple[str, int]:
    """Couple passive DIP joints to their driving PIP/MCP joints.

    The real GHand fingers use a tendon/linkage coupling so the distal
    interphalangeal joint moves together with the proximal joint. The source
    URDF models PIP and DIP as independent revolute joints, which makes the
    DIP hang limp in Gazebo. Insert <mimic> tags so each DIP tracks its
    driver joint.
    """
    root = ET.fromstring(urdf_str)
    added = 0
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        prefix = ""
        canonical_name = name
        for candidate in ("left_", "right_"):
            if name.startswith(candidate):
                prefix = candidate
                canonical_name = name[len(candidate):]
                break
        if canonical_name not in mimic_map:
            continue
        if joint.find("mimic") is not None:
            continue
        driver, multiplier, offset = mimic_map[canonical_name]
        mimic = ET.SubElement(joint, "mimic")
        mimic.set("joint", prefix + driver)
        mimic.set("multiplier", str(multiplier))
        mimic.set("offset", str(offset))
        added += 1
    return ET.tostring(root, encoding="unicode"), added


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Process a URDF/xacro for Gazebo and apply a small negative "
            "tolerance to zero lower joint limits."
        )
    )
    parser.add_argument("urdf_path", help="Path to URDF or xacro file")
    parser.add_argument(
        "--limit-tolerance",
        type=float,
        default=-0.01,
        help=(
            "Negative tolerance to apply to zero lower limits "
            "(default: -0.01)"
        ),
    )
    parser.add_argument(
        "--xacro-arg",
        action="append",
        default=[],
        help="Additional argument to pass to xacro (can be repeated)",
    )
    parser.add_argument("--mimic-config", required=True)
    parser.add_argument("--ghand5-dynamics", action="store_true")
    args = parser.parse_args()

    xacro_cmd = ["xacro", args.urdf_path]
    for arg in args.xacro_arg:
        xacro_cmd.append(arg)

    result = subprocess.run(xacro_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        sys.exit(result.returncode)

    patched_urdf, _ = patch_lower_limits(
        result.stdout, args.limit_tolerance, ghand5_dynamics=args.ghand5_dynamics,
    )
    with open(args.mimic_config, encoding="utf-8") as source:
        mimic_map = yaml.safe_load(source)
    patched_urdf, _ = add_mimic_joints(patched_urdf, mimic_map)
    print(patched_urdf)


if __name__ == "__main__":
    main()
