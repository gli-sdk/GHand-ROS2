#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Combine the left and right GHand URDFs into one prefixed robot."""

import argparse
import copy
import xml.etree.ElementTree as ET


def prefix_hand(robot, prefix):
    """Copy robot children and prefix link and joint references."""
    children = []
    for original in robot:
        element = copy.deepcopy(original)
        for item in element.iter():
            if item.tag in ("link", "joint") and "name" in item.attrib:
                item.set("name", prefix + item.get("name"))
            if item.tag in ("parent", "child") and "link" in item.attrib:
                item.set("link", prefix + item.get("link"))
            if item.tag == "mimic" and "joint" in item.attrib:
                item.set("joint", prefix + item.get("joint"))
            if item.tag == "gazebo" and "reference" in item.attrib:
                item.set("reference", prefix + item.get("reference"))
        children.append(element)
    return children


def mount_joint(side, x_offset):
    """Create a fixed mount that keeps both palms at the same height.

    The Gazebo launch rotates the complete hand model before placing it above
    the table.  With that rotation, the robot's local X axis becomes the
    world's horizontal Y axis, while its local Y axis becomes world Z.  Place
    the separation on local X so the two hands stay side by side instead of
    ending up one above the other.
    """
    joint = ET.Element("joint", name=f"{side}_mount_joint", type="fixed")
    ET.SubElement(joint, "origin", xyz=f"{x_offset} 0 0", rpy="0 0 0")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link=f"{side}_base_link")
    return joint


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", required=True)
    parser.add_argument("--right", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--separation", type=float, default=0.36)
    parser.add_argument("--product-name", default="ghand5")
    args = parser.parse_args()

    left_robot = ET.parse(args.left).getroot()
    right_robot = ET.parse(args.right).getroot()

    dual = ET.Element("robot", name=f"{args.product_name}-dual")
    dual.append(ET.Element("link", name="world"))

    half_separation = args.separation / 2.0
    for element in prefix_hand(left_robot, "left_"):
        dual.append(element)
    for element in prefix_hand(right_robot, "right_"):
        dual.append(element)

    dual.append(mount_joint("left", half_separation))
    dual.append(mount_joint("right", -half_separation))

    ET.indent(dual, space="  ")
    ET.ElementTree(dual).write(
        args.output,
        encoding="utf-8",
        xml_declaration=True,
    )


if __name__ == "__main__":
    main()
