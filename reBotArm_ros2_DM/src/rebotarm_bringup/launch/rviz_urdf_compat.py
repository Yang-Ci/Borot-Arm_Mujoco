#!/usr/bin/env python3
"""Adapt multi-material URDF links for the ROS 2 Jazzy RViz renderer.

RViz can resolve every entry in a link's ``visual_array`` to that link's
first material.  Moving additional visuals to zero-offset fixed child links
keeps the same source URDF, geometry, poses, and shared colours while giving
RViz one unambiguous material per rendered link.
"""

import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET


def _visual_suffix(visual: ET.Element, index: int) -> str:
    mesh = visual.find("geometry/mesh")
    filename = mesh.get("filename", "") if mesh is not None else ""
    stem = Path(filename).stem or f"part_{index}"
    return re.sub(r"[^A-Za-z0-9_]+", "_", stem).strip("_") or f"part_{index}"


def make_rviz_compatible(urdf_xml: str) -> str:
    """Return an equivalent URDF with at most one visual per link."""
    root = ET.fromstring(urdf_xml)
    additions = []
    used_links = {link.get("name", "") for link in root.findall("link")}
    used_joints = {joint.get("name", "") for joint in root.findall("joint")}

    for link in list(root.findall("link")):
        visuals = link.findall("visual")
        if len(visuals) <= 1:
            continue

        parent_name = link.get("name", "")
        for index, visual in enumerate(visuals[1:], start=1):
            suffix = _visual_suffix(visual, index)
            child_name = f"{parent_name}__rviz_visual_{suffix}"
            joint_name = f"{child_name}__fixed_joint"
            serial = 2
            while child_name in used_links or joint_name in used_joints:
                child_name = f"{parent_name}__rviz_visual_{suffix}_{serial}"
                joint_name = f"{child_name}__fixed_joint"
                serial += 1

            used_links.add(child_name)
            used_joints.add(joint_name)
            link.remove(visual)

            child_link = ET.Element("link", {"name": child_name})
            child_link.append(visual)
            fixed_joint = ET.Element("joint", {"name": joint_name, "type": "fixed"})
            ET.SubElement(fixed_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            ET.SubElement(fixed_joint, "parent", {"link": parent_name})
            ET.SubElement(fixed_joint, "child", {"link": child_name})
            additions.extend((child_link, fixed_joint))

    root.extend(additions)
    return ET.tostring(root, encoding="unicode")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} ROBOT.urdf", file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    print(make_rviz_compatible(source.read_text(encoding="utf-8")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
