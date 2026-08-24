#!/usr/bin/env python3
"""Read arm-link material colours straight out of the generated DAE files."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "src" / "rebotarm_bringup" / "description" / "meshes"

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def effect_diffuse(effect_el: ET.Element) -> list[str] | None:
    diffuse = effect_el.find(".//c:diffuse/c:color", NS)
    if diffuse is not None and diffuse.text:
        return diffuse.text.split()
    return None


def read_dae_colors(path: Path) -> dict[str, str]:
    tree = ET.parse(path)
    root = tree.getroot()
    colors: dict[str, str] = {}

    effects: dict[str, ET.Element] = {}
    for effect in root.findall(".//c:effect", NS):
        eid = effect.get("id", "")
        effects[eid] = effect

    for mat in root.findall(".//c:material", NS):
        name = mat.get("name") or mat.get("id") or ""
        name = name.replace("-material", "")
        inst = mat.find("c:instance_effect", NS)
        if inst is None:
            continue
        url = inst.get("url", "").lstrip("#")
        effect = effects.get(url)
        if effect is None:
            sys.stderr.write(f"  !! {path.name}: effect {url} not found\n")
            continue
        rgba = effect_diffuse(effect)
        if rgba is None:
            sys.stderr.write(f"  !! {path.name}: {name} has no diffuse colour\n")
            continue
        colors[name] = " ".join(f"{float(v):g}" for v in rgba)

    return colors


def main() -> None:
    result: dict[str, dict[str, str]] = {}
    for dae in sorted(MESH_DIR.glob("colored_link*.dae")):
        key = dae.name.replace("colored_", "").replace(".dae", "")
        result[key] = read_dae_colors(dae)

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()