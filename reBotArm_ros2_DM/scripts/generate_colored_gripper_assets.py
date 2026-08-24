#!/usr/bin/env python3
"""Generate the shared RViz/MuJoCo coloured gripper visual meshes.

The source binary STL files contain four contiguous CAD solids per finger.
The rack-and-pinion mechanism crosses the racks, so each moving finger visual
uses its own body solids and the rack geometry from the opposite source STL.
Collision meshes are intentionally not generated or changed here.
"""

from __future__ import annotations

import struct
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "src" / "rebotarm_bringup" / "description" / "meshes"

BASE_RANGES = {
    "seeed_yellow": [("gripper_base.stl", 0, 6734)],
    "metal": [("gripper_base.stl", 6734, 2628)],
    "hardware_black": [("gripper_base.stl", 9362, 2484)],
}

FINGER_RANGES = {
    "finger_black": (2148, 1142),
    "travel_stop_yellow": (3290, 1080),
    "carriage_grey": (4370, 400),
    "rack_metal": (0, 2148),
}


def read_binary_stl(path: Path) -> list[bytes]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(
            f"Expected binary STL size {expected_size}, got {len(data)}: {path}"
        )
    return [data[84 + index * 50 : 84 + (index + 1) * 50] for index in range(triangle_count)]


def write_binary_stl(path: Path, triangles: list[bytes], label: str) -> None:
    header = f"reBot coloured visual: {label}".encode("ascii")[:80].ljust(80, b" ")
    payload = header + struct.pack("<I", len(triangles)) + b"".join(triangles)
    path.write_bytes(payload)
    print(f"{path.name}: {len(triangles)} triangles")


def take(records: list[bytes], start: int, count: int) -> list[bytes]:
    end = start + count
    if start < 0 or end > len(records):
        raise ValueError(f"Triangle range {start}:{end} exceeds {len(records)}")
    return records[start:end]


def main() -> None:
    source = {
        name: read_binary_stl(MESH_DIR / name)
        for name in ("gripper_base.stl", "left_finger.stl", "right_finger.stl")
    }

    for finish, ranges in BASE_RANGES.items():
        triangles: list[bytes] = []
        for source_name, start, count in ranges:
            triangles.extend(take(source[source_name], start, count))
        write_binary_stl(
            MESH_DIR / f"colored_gripper_base_{finish}.stl",
            triangles,
            f"gripper base {finish}",
        )

    for side in ("left", "right"):
        body_source = source[f"{side}_finger.stl"]
        opposite = "right" if side == "left" else "left"
        rack_source = source[f"{opposite}_finger.stl"]

        for finish, (start, count) in FINGER_RANGES.items():
            records = rack_source if finish == "rack_metal" else body_source
            write_binary_stl(
                MESH_DIR / f"colored_{side}_finger_{finish}.stl",
                take(records, start, count),
                f"{side} finger {finish}",
            )


if __name__ == "__main__":
    main()
