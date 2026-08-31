#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Generate prefixed dual-hand MoveIt semantic and limit configurations."""

import argparse
import copy
import xml.etree.ElementTree as ET


JOINT_NAMES = (
    "Thumb_IP",
    "Thumb_MCP",
    "Thumb_TMC_FE",
    "Thumb_TMC_AA",
    "Thumb_TMC_PS",
    "FF_DIP",
    "FF_PIP",
    "FF_MCP",
    "FF_MCP_AA",
    "MF_DIP",
    "MF_PIP",
    "MF_MCP",
    "RF_DIP",
    "RF_PIP",
    "RF_MCP",
    "LF_DIP",
    "LF_PIP",
    "LF_MCP",
)

KINEMATIC_GROUPS = (
    "index_finger",
    "middle_finger",
    "ring_finger",
    "little_finger",
    "thumb",
)


def prefixed_srdf_children(source, prefix):
    """Prefix all link, joint, group, and end-effector references."""
    result = []
    for original in source:
        element = copy.deepcopy(original)
        if element.tag == "group":
            element.set("name", prefix + element.get("name"))
            for item in element.iter():
                if (
                    item is not element
                    and item.tag == "group"
                    and "name" in item.attrib
                ):
                    item.set("name", prefix + item.get("name"))
                if item.tag == "link" and "name" in item.attrib:
                    item.set("name", prefix + item.get("name"))
                if (
                    item.tag in ("joint", "passive_joint")
                    and "name" in item.attrib
                ):
                    item.set("name", prefix + item.get("name"))
                if item.tag == "chain":
                    item.set("base_link", prefix + item.get("base_link"))
                    item.set("tip_link", prefix + item.get("tip_link"))
        elif element.tag == "group_state":
            element.set("group", prefix + element.get("group"))
            for joint in element.findall("joint"):
                joint.set("name", prefix + joint.get("name"))
        elif element.tag == "end_effector":
            element.set("name", prefix + element.get("name"))
            for attribute in ("parent_link", "group", "parent_group"):
                if attribute in element.attrib:
                    element.set(attribute, prefix + element.get(attribute))
        elif element.tag == "disable_collisions":
            element.set("link1", prefix + element.get("link1"))
            element.set("link2", prefix + element.get("link2"))
        elif element.tag == "passive_joint":
            element.set("name", prefix + element.get("name"))
        result.append(element)
    return result


def dual_group_state(name, left_source, right_source):
    state = ET.Element("group_state", name=name, group="dual_hands")
    for source, prefix in ((left_source, "left_"), (right_source, "right_")):
        source_state = source.find(
            f"group_state[@name='{name}'][@group='hand']"
        )
        if source_state is None:
            raise RuntimeError(f"Missing hand/{name} group state")
        for original_joint in source_state.findall("joint"):
            joint = copy.deepcopy(original_joint)
            joint.set("name", prefix + joint.get("name"))
            state.append(joint)
    return state


def write_joint_limits(path):
    lines = [
        "default_velocity_scaling_factor: 0.2",
        "default_acceleration_scaling_factor: 0.2",
        "joint_limits:",
    ]
    for prefix in ("left_", "right_"):
        for name in JOINT_NAMES:
            lines.extend(
                [
                    f"  {prefix}{name}:",
                    "    has_velocity_limits: true",
                    "    max_velocity: 100.0",
                    "    has_acceleration_limits: true",
                    "    max_acceleration: 100.0",
                ]
            )
    with open(path, "w", encoding="utf-8") as output:
        output.write("\n".join(lines) + "\n")


def write_kinematics(path):
    lines = []
    for prefix in ("left_", "right_"):
        for group in KINEMATIC_GROUPS:
            lines.extend(
                [
                    f"{prefix}{group}:",
                    "  kinematics_solver: "
                    "kdl_kinematics_plugin/KDLKinematicsPlugin",
                    "  kinematics_solver_search_resolution: 0.005",
                    "  kinematics_solver_timeout: 0.005",
                    "",
                ]
            )
    with open(path, "w", encoding="utf-8") as output:
        output.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-srdf", required=True)
    parser.add_argument("--right-srdf", required=True)
    parser.add_argument("--srdf-output", required=True)
    parser.add_argument("--joint-limits-output", required=True)
    parser.add_argument("--kinematics-output", required=True)
    args = parser.parse_args()

    left = ET.parse(args.left_srdf).getroot()
    right = ET.parse(args.right_srdf).getroot()
    dual = ET.Element("robot", name="ghand5-dual")

    for element in prefixed_srdf_children(left, "left_"):
        dual.append(element)
    for element in prefixed_srdf_children(right, "right_"):
        dual.append(element)

    dual_group = ET.Element("group", name="dual_hands")
    ET.SubElement(dual_group, "group", name="left_hand")
    ET.SubElement(dual_group, "group", name="right_hand")
    dual.insert(0, dual_group)
    dual.append(dual_group_state("open", left, right))
    dual.append(dual_group_state("fist", left, right))

    ET.indent(dual, space="  ")
    ET.ElementTree(dual).write(
        args.srdf_output,
        encoding="utf-8",
        xml_declaration=True,
    )
    write_joint_limits(args.joint_limits_output)
    write_kinematics(args.kinematics_output)


if __name__ == "__main__":
    main()
