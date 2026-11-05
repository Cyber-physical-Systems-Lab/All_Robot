#!/usr/bin/env python3
"""Normalise URDFs so that the SAME file works in RViz, Gazebo, MuJoCo and Isaac Sim.

What it does (idempotent, every change is listed in the report):
  1. Rewrites every <mesh filename> to  package://cps_robot_description/robots/<r>/meshes/<file>
     (legacy styles handled: package://robot_description/..., package://myagv_urdf/urdf/...,
      package://mycobot_description/urdf/mycobot/..., absolute /home/... paths).
  2. Removes xacro-only elements (<xacro:property>) from plain .urdf files.
  3. Adds a placeholder <inertial> to links that have geometry but no inertial
     (Gazebo silently drops such links, MuJoCo/PhysX need mass).
  4. Copies <visual> geometry to <collision> when a link has none.
  5. Replaces zero/missing effort and velocity limits on movable joints.

Usage:
  python3 scripts/normalize_urdf.py            # all robots in robots.yaml
  python3 scripts/normalize_urdf.py nova5 go2  # selected robots
  python3 scripts/normalize_urdf.py --check    # report only, do not write
"""
from __future__ import annotations

import argparse
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (REPO, XACRO_NS, load_registry, mesh_elements, parse_urdf,  # noqa: E402
                    resolve_uri, robot_urdfs, to_package_uri, write_urdf)

PLACEHOLDER_TAG = "placeholder inertial added by normalize_urdf.py - replace with CAD values"


def build_mesh_index() -> dict[str, list[Path]]:
    idx: dict[str, list[Path]] = {}
    for p in (REPO / "robots").glob("*/meshes/**/*"):
        if p.is_file() and "mujoco" not in p.parts:
            idx.setdefault(p.name, []).append(p)
    return idx


def find_mesh(uri: str, search: list[str], idx: dict[str, list[Path]]) -> Path | None:
    p = resolve_uri(uri)
    if p is not None and p.is_file() and REPO in p.resolve().parents:
        return p
    name = uri.replace("\\", "/").rsplit("/", 1)[-1]
    cands = idx.get(name, [])
    if not cands:  # tolerate case differences, e.g. .stl vs .STL
        cands = [c for k, v in idx.items() if k.lower() == name.lower() for c in v]
    for robot in search:  # preferred folder order from robots.yaml
        for c in cands:
            if c.relative_to(REPO).parts[1] == robot:
                return c
    return cands[0] if cands else None


def normalize(path: Path, cfg: dict, idx, write: bool = True) -> list[str]:
    notes: list[str] = []
    tree = parse_urdf(path)
    root = tree.getroot()
    d = cfg.get("defaults", {})

    # 1. mesh paths
    for m in mesh_elements(root):
        old = m.attrib["filename"]
        new_path = find_mesh(old, cfg.get("mesh_search", []), idx)
        if new_path is None:
            notes.append(f"UNRESOLVED mesh {old}")
            continue
        new = to_package_uri(new_path)
        if new != old:
            m.attrib["filename"] = new
            notes.append(f"mesh {old} -> {new}")

    # 2. xacro leftovers in a plain URDF
    for parent in list(root.iter()):
        for child in list(parent):
            if isinstance(child.tag, str) and child.tag.startswith("{%s}" % XACRO_NS):
                parent.remove(child)
                notes.append(f"removed xacro element <{child.tag.split('}')[1]}>")

    # 3 + 4. inertial / collision
    for link in root.findall("link"):
        name = link.get("name")
        has_geom = link.find("visual") is not None or link.find("collision") is not None
        if has_geom and link.find("collision") is None:
            for vis in link.findall("visual"):
                col = ET.SubElement(link, "collision")
                for tag in ("origin", "geometry"):
                    if vis.find(tag) is not None:
                        col.append(copy.deepcopy(vis.find(tag)))
            notes.append(f"link {name}: collision copied from visual")
        if has_geom and link.find("inertial") is None:
            m = float(d.get("link_mass", 0.1))
            i = 2.0 / 5.0 * m * 0.03 ** 2  # solid sphere r=3 cm
            link.insert(0, ET.Comment(f" {PLACEHOLDER_TAG} "))
            inert = ET.Element("inertial")
            ET.SubElement(inert, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(inert, "mass", value=f"{m:g}")
            ET.SubElement(inert, "inertia", ixx=f"{i:.3e}", ixy="0", ixz="0",
                          iyy=f"{i:.3e}", iyz="0", izz=f"{i:.3e}")
            link.insert(1, inert)
            notes.append(f"link {name}: placeholder inertial mass={m:g} kg")

    # 5. joint limits
    for j in root.findall("joint"):
        if j.get("type") not in ("revolute", "prismatic", "continuous"):
            continue
        lim = j.find("limit")
        if lim is None:
            lim = ET.SubElement(j, "limit")
            if j.get("type") != "continuous":
                lim.set("lower", "-3.14159")
                lim.set("upper", "3.14159")
        for key in ("effort", "velocity"):
            try:
                val = float(lim.get(key, "0"))
            except ValueError:
                val = 0.0
            if val <= 0.0:
                lim.set(key, f"{float(d.get(key, 1.0)):g}")
                notes.append(f"joint {j.get('name')}: {key} {lim.get(key)} (was {val:g})")

    if write and notes:
        write_urdf(tree, path)
    return notes


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robots", nargs="*")
    ap.add_argument("--check", action="store_true", help="only report")
    ap.add_argument("--report", type=Path, default=None, help="write markdown report")
    a = ap.parse_args()

    reg = load_registry()["robots"]
    idx = build_mesh_index()
    lines, unresolved = ["# URDF normalisation report", ""], 0
    for name in a.robots or reg:
        cfg = reg[name]
        for urdf in robot_urdfs(cfg):
            if not urdf.exists():
                lines.append(f"## {name} — MISSING {urdf.relative_to(REPO)}\n")
                continue
            notes = normalize(urdf, cfg, idx, write=not a.check)
            unresolved += sum(n.startswith("UNRESOLVED") for n in notes)
            lines.append(f"## {name} — `{urdf.relative_to(REPO)}` ({len(notes)} changes)\n")
            lines += [f"- {n}" for n in notes] + [""]
    text = "\n".join(lines)
    if a.report:
        a.report.write_text(text)
    print(text if a.check else f"normalised; {unresolved} unresolved meshes"
          + (f"; report: {a.report}" if a.report else ""))
    return 1 if unresolved else 0


if __name__ == "__main__":
    sys.exit(main())
