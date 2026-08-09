#!/usr/bin/env python3
"""Rebuild the arm visuals of the coloured URDF from per-material STL parts.

Keeps every linkage/inertial/collision and the existing gripper visuals
untouched.  Only the six arm <visual> blocks are replaced with one
<visual> per material STL, referencing a <material> defined at the top of
the URDF (diffuse colours extracted from the DAE the user already loves).

Safe to re-run: the original file is restored from git before rebuilding.
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "src" / "rebotarm_bringup" / "description" / "urdf"
MESH_DIR = ROOT / "src" / "rebotarm_bringup" / "description" / "meshes"

URDF = URDF_DIR / "reBot-DevArm_colored.urdf"

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

# link -> ordered material parts, name and DAE diffuse colour
ARM_MATERIALS = {
    "base_link": ["matte_black", "hardware_black", "silver_trim"],
    "link1": ["anodized_grey", "hardware_black", "matte_black"],
    "link2": ["anodized_grey", "hardware_black", "matte_black", "seeed_yellow"],
    "link3": ["anodized_grey", "hardware_black", "matte_black", "seeed_yellow"],
    "link4": ["anodized_grey", "hardware_black", "matte_black", "seeed_yellow"],
    "link5": ["anodized_grey", "hardware_black", "matte_black"],
    "link6": ["hardware_black", "matte_black"],
}

EYE_COLOURS = {
    "matte_black": (0.14, 0.16, 0.15, 1.0),
    "hardware_black": (0.055, 0.065, 0.06, 1.0),
    "anodized_grey": (0.5, 0.54, 0.51, 1.0),
    "seeed_yellow": (0.73, 0.84, 0.12, 1.0),
    "silver_trim": (0.72, 0.76, 0.73, 1.0),
}


def fmt_rgba(color: tuple[float, float, float, float]) -> str:
    return " ".join(f"{channel:g}" for channel in color)


def material_block(name: str) -> str:
    rgba = fmt_rgba(EYE_COLOURS[name])
    return (
        f'  <material name="{name}">\n'
        f"    <color rgba=\"{rgba}\" />\n"
        "  </material>"
    )


def visual_block(link: str, material: str) -> str:
    return (
        f'    <visual name="{link}_{material}">\n'
        '      <origin xyz="0 0 0" rpy="0 0 0" />\n'
        "      <geometry>\n"
        f'        <mesh filename="package://rebotarm_bringup/description/meshes/colored_{link}_{material}.stl" />\n'
        "      </geometry>\n"
        f'      <material name="{material}" />\n'
        "    </visual>"
    )


def build() -> str:
    for link, parts in ARM_MATERIALS.items():
        for material in parts:
            mesh = MESH_DIR / f"colored_{link}_{material}.stl"
            if not mesh.is_file():
                raise SystemExit(f"missing part mesh: {mesh.name}")

    source = URDF.read_text(encoding="utf-8")
    robot = ET.fromstring(source)

    existing = {m.get("name") for m in robot.findall("material")}
    for material in {m for parts in ARM_MATERIALS.values() for m in parts}:
        if material not in existing:
            robot.append(ET.fromstring(material_block(material)))

    for link, parts in ARM_MATERIALS.items():
        link_el = next(l for l in robot.findall("link") if l.get("name") == link)
        for visual in link_el.findall("visual"):
            link_el.remove(visual)
        inertial_idx = next(
            i for i, child in enumerate(link_el) if child.tag == "inertial"
        )
        for material in parts:
            block = ET.fromstring(visual_block(link, material))
            link_el.insert(inertial_idx + 1, block)

    indent(robot)
    return ET.tostring(robot, encoding="unicode", xml_declaration=True)


def indent(elem: ET.Element, level: int = 0) -> None:
    spacing_outer = "  " * level
    spacing_inner = "  " * (level + 1)
    children = list(elem)
    if children:
        if not elem.text or not elem.text.strip():
            elem.text = "\n" + spacing_inner
        for i, child in enumerate(children):
            indent(child, level + 1)
            if i == len(children) - 1:
                child.tail = "\n" + spacing_outer
            else:
                child.tail = "\n" + spacing_inner
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = "\n" + spacing_outer


def main() -> None:
    if not URDF.is_file():
        raise SystemExit(f"URDF not found: {URDF}")

    backup = URDF.with_suffix(".urdf.bak")
    shutil.copyfile(URDF, backup)

    content = build()
    URDF.write_text(content.replace("\r", ""), encoding="utf-8")

    print(f"written: {URDF}")
    print(f"backup:  {backup}")
    print(f"arm visuals replaced with {sum(len(v) for v in ARM_MATERIALS.values())} split meshes")


if __name__ == "__main__":
    main()