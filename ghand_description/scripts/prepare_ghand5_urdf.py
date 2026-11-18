#!/usr/bin/env python3
"""Installable GHand5 model with package mesh paths and an inertialess root."""

import argparse
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output")
    args = parser.parse_args()

    model = ET.parse(args.source)
    robot = model.getroot()
    for mesh in robot.iter("mesh"):
        mesh.set("filename", mesh.attrib["filename"].replace(
            "ghand5_system/meshes/", "package://ghand_description/meshes/ghand5/", 1,
        ))
    robot.insert(0, ET.Element("link", name="ghand_root"))
    joint = ET.SubElement(robot, "joint", name="ghand_root_joint", type="fixed")
    ET.SubElement(joint, "parent", link="ghand_root")
    ET.SubElement(joint, "child", link="base_link")
    ET.indent(model, space="  ")
    model.write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
