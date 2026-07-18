#!/usr/bin/env python3
"""Port the CRANE-X7 URDF (xacro) to a roki .ztk model.

Pipeline:
  1. Copy crane_x7_description/urdf into a work dir, replacing
     $(find crane_x7_description) with the work dir path — standalone pip
     xacro cannot resolve $(find) without a ROS package index.
  2. Expand xacro to a flat URDF (gazebo/D435 off, mock components on).
  3. Rewrite package:// mesh URIs to repo-relative paths and copy the
     STL meshes into models/crane_x7/meshes.
  4. Convert with urdf2ztk (roki).
  5. urdf2ztk drops effort limits and motors entirely: generate one
     [roki::motor] (type trq, max/min = ±effort from the URDF) per driven
     joint and insert a `motor:` key into the matching [roki::link]
     section. finger_b gets a motor too — it has no servo on the real
     robot, but the simulator applies the mimic coupling torque through it.
  6. Append tools/crane_x7_motor.patch.ztk verbatim (hand-tuned overrides,
     e.g. identified friction parameters; empty until M7).

Run from the repo root:
  uv run --project tools tools/port_model.py
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DESC = REPO / "third_party" / "crane_x7_description"
WORK = REPO / "build" / "model_port"
OUT_DIR = REPO / "models" / "crane_x7"
PATCH = REPO / "tools" / "crane_x7_motor.patch.ztk"

XACRO_ARGS = ["use_gazebo:=false", "use_d435:=false", "use_mock_components:=true"]
MESH_URI_PREFIX = "package://crane_x7_description/meshes"
MESH_REL_PATH = "meshes"  # relative to the .ztk itself: mi-lib viewers
# (rk_pen/rk_anim) chdir into the model's directory before reading, and
# ChainModel mirrors that convention


def expand_xacro() -> str:
    """Return the flat URDF text."""
    urdf_src = WORK / "crane_x7_description" / "urdf"
    if urdf_src.parent.exists():
        shutil.rmtree(urdf_src.parent)
    urdf_src.mkdir(parents=True)
    for f in (DESC / "urdf").iterdir():
        text = f.read_text()
        text = text.replace("$(find crane_x7_description)", str(urdf_src.parent))
        (urdf_src / f.name).write_text(text)

    import xacro  # deferred: clearer error if run outside `uv run`

    doc = xacro.process_file(
        str(urdf_src / "crane_x7.urdf.xacro"),
        mappings=dict(arg.split(":=", 1) for arg in XACRO_ARGS),
    )
    return doc.toprettyxml(indent="  ")


def rewrite_meshes(urdf_text: str) -> str:
    for sub in ("visual", "collision"):
        src = DESC / "meshes" / sub
        dst = OUT_DIR / "meshes" / sub
        dst.mkdir(parents=True, exist_ok=True)
        for stl in src.glob("*.stl"):
            shutil.copy2(stl, dst / stl.name)
    return urdf_text.replace(MESH_URI_PREFIX, MESH_REL_PATH)


def driven_joints(urdf_text: str) -> list[dict]:
    """Revolute joints from the flat URDF: name, child link, limits."""
    root = ET.fromstring(urdf_text)
    joints = []
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        child = joint.find("child")
        if name is None or limit is None or child is None:
            sys.exit(f"error: revolute joint {name!r} lacks <limit> or <child>")
        joints.append(
            {
                "name": name,
                "child": child.get("link"),
                "effort": float(limit.get("effort", "nan")),
                "velocity": float(limit.get("velocity", "nan")),
                "lower": float(limit.get("lower", "nan")),
                "upper": float(limit.get("upper", "nan")),
            }
        )
    return joints


def convert_to_ztk(urdf_text: str) -> str:
    urdf2ztk = shutil.which("urdf2ztk") or str(Path.home() / "usr/bin/urdf2ztk")
    urdf_file = WORK / "crane_x7.urdf"
    urdf_file.write_text(urdf_text)
    subprocess.run([urdf2ztk, str(urdf_file)], check=True, cwd=WORK)
    ztk_file = urdf_file.with_suffix(".ztk")
    if not ztk_file.exists():
        sys.exit(f"error: urdf2ztk produced no output at {ztk_file}")
    return ztk_file.read_text()


def merge_motors(ztk_text: str, joints: list[dict]) -> str:
    # one trq motor per driven joint, bounded by the URDF effort limit
    motor_sections = []
    for j in joints:
        motor_sections.append(
            f"\n[roki::motor]\n"
            f"name: {j['name']}_motor\n"
            f"type: trq\n"
            f"max: {j['effort']}\n"
            f"min: {-j['effort']}\n"
        )

    # insert `motor:` after the jointtype line of each driven link section
    child_to_motor = {j["child"]: f"{j['name']}_motor" for j in joints}
    lines = ztk_text.splitlines(keepends=True)
    out: list[str] = []
    current_link = None
    for i, line in enumerate(lines):
        out.append(line)
        if line.strip() == "[roki::link]":
            # link name follows the tag
            for peek in lines[i + 1 : i + 3]:
                m = re.match(r"\s*name\s*:\s*(\S+)", peek)
                if m:
                    current_link = m.group(1)
                    break
        m = re.match(r"\s*jointtype\s*:", line)
        if m and current_link in child_to_motor:
            out.append(f"motor: {child_to_motor[current_link]}\n")
    merged = "".join(out) + "".join(motor_sections)

    if PATCH.exists() and PATCH.read_text().strip():
        merged += "\n" + PATCH.read_text()
    return merged


def main() -> None:
    print(f"expanding xacro ({', '.join(XACRO_ARGS)})")
    urdf_text = expand_xacro()
    urdf_text = rewrite_meshes(urdf_text)
    joints = driven_joints(urdf_text)
    print(f"driven joints: {[j['name'] for j in joints]}")

    print("converting with urdf2ztk")
    ztk_text = convert_to_ztk(urdf_text)
    ztk_text = merge_motors(ztk_text, joints)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "crane_x7.ztk"
    out.write_text(ztk_text)
    print(f"wrote {out.relative_to(REPO)}")
    print("verify with:  rk_pen models/crane_x7/crane_x7.ztk")


if __name__ == "__main__":
    main()
