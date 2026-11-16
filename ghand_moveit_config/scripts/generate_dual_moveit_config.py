#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Generate prefixed dual-hand MoveIt semantic and limit configurations."""

import argparse
import copy
import xml.etree.ElementTree as ET

import yaml


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
        elif element.tag == "virtual_joint":
            # Both vendor hands are mounted below the shared URDF world link.
            continue
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


def write_joint_limits(path, source_path):
    with open(source_path, encoding="utf-8") as source:
        config = yaml.safe_load(source)
    config["joint_limits"] = {
        prefix + name: copy.deepcopy(limits)
        for prefix in ("left_", "right_")
        for name, limits in config["joint_limits"].items()
    }
    with open(path, "w", encoding="utf-8") as output:
        yaml.safe_dump(config, output, sort_keys=False)


def write_kinematics(path, source_path):
    with open(source_path, encoding="utf-8") as source:
        config = yaml.safe_load(source)
    config = {
        prefix + group: copy.deepcopy(settings)
        for prefix in ("left_", "right_")
        for group, settings in config.items()
    }
    with open(path, "w", encoding="utf-8") as output:
        yaml.safe_dump(config, output, sort_keys=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-srdf", required=True)
    parser.add_argument("--right-srdf", required=True)
    parser.add_argument("--joint-limits", required=True)
    parser.add_argument("--kinematics", required=True)
    parser.add_argument("--srdf-output", required=True)
    parser.add_argument("--joint-limits-output", required=True)
    parser.add_argument("--kinematics-output", required=True)
    args = parser.parse_args()

    left = ET.parse(args.left_srdf).getroot()
    right = ET.parse(args.right_srdf).getroot()
    product_name = left.get("name").removesuffix("-left")
    if right.get("name") != f"{product_name}-right":
        raise ValueError("Left and right SRDFs must describe the same product")
    dual = ET.Element("robot", name=f"{product_name}-dual")
    ET.SubElement(dual, "virtual_joint", name="world_joint", type="fixed",
                  parent_frame="world", child_link="world")

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
    write_joint_limits(args.joint_limits_output, args.joint_limits)
    write_kinematics(args.kinematics_output, args.kinematics)


if __name__ == "__main__":
    main()
