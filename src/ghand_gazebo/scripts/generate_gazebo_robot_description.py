#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: MIT

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


def patch_lower_limits(
    urdf_str: str, epsilon: float = -0.01
) -> tuple[str, int]:
    """Patch zero lower joint limits to the given negative tolerance."""
    root = ET.fromstring(urdf_str)
    patched = 0
    for joint in root.iter("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
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
    args = parser.parse_args()

    xacro_cmd = ["xacro", args.urdf_path]
    for arg in args.xacro_arg:
        xacro_cmd.append(arg)

    result = subprocess.run(xacro_cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        sys.exit(result.returncode)

    patched_urdf, count = patch_lower_limits(
        result.stdout, args.limit_tolerance
    )
    print(
        f"Patched {count} joint limit(s) with lower=0 -> "
        f"lower={args.limit_tolerance}",
        file=sys.stderr,
    )
    print(patched_urdf)


if __name__ == "__main__":
    main()
