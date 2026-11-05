#!/usr/bin/env python3
"""Validate every robot model. Run before every commit / in CI.

  python3 scripts/validate.py              # static checks (no simulator needed)
  python3 scripts/validate.py --mujoco     # + load & step MJCF       (pip install mujoco)
  python3 scripts/validate.py --gazebo     # + URDF->SDF via `ign sdf` (Gazebo Fortress)
  python3 scripts/validate.py --all --markdown docs/status.md

Isaac Sim / Isaac Lab is checked separately (needs a GPU):
  ./isaaclab.sh -p isaaclab/spawn_robot.py --robot all --check --headless

Static checks per URDF: XML parses, single root + tree structure, every mesh exists,
every moving link has positive mass + physically valid inertia, movable joints have
effort/velocity > 0, standalone URDF is up to date, no absolute /home paths.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (REPO, load_registry, mesh_elements, parse_urdf, resolve_uri,  # noqa: E402
                    robot_urdfs, standalone_path)

OK, WARN, FAIL = "PASS", "WARN", "FAIL"


def check_urdf(path: Path) -> tuple[list[str], list[str]]:
    errs, warns = [], []
    try:
        root = parse_urdf(path).getroot()
    except Exception as e:  # noqa: BLE001
        return [f"XML parse error: {e}"], []
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    children, parents = set(), set()
    for j in joints:
        p, c = j.find("parent"), j.find("child")
        if p is None or c is None:
            errs.append(f"joint {j.get('name')} missing parent/child")
            continue
        for ln in (p.get("link"), c.get("link")):
            if ln not in links:
                errs.append(f"joint {j.get('name')} references unknown link {ln}")
        if c.get("link") in children:
            errs.append(f"link {c.get('link')} has two parents")
        children.add(c.get("link"))
        parents.add(p.get("link"))
    fixed_children = {j.find("child").get("link") for j in joints
                      if j.get("type") == "fixed" and j.find("child") is not None}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        errs.append(f"expected 1 root link, found {roots}")

    for m in mesh_elements(root):
        uri = m.get("filename")
        if uri.startswith("/home") or uri.startswith("file:///home"):
            errs.append(f"absolute path {uri}")
        p = resolve_uri(uri, path.parent)
        if p is None or not p.is_file():
            errs.append(f"missing mesh {uri}")

    for name, l in links.items():
        has_geom = l.find("visual") is not None or l.find("collision") is not None
        inert = l.find("inertial")
        if inert is None:
            if has_geom:
                errs.append(f"link {name} has geometry but no <inertial>")
            continue
        try:
            mass = float(inert.find("mass").get("value"))
            I = {k: float(inert.find("inertia").get(k)) for k in ("ixx", "iyy", "izz")}
        except Exception:  # noqa: BLE001
            errs.append(f"link {name} malformed <inertial>")
            continue
        if mass <= 0:
            if name in fixed_children:  # massless sensor frame, lumped into parent by simulators
                warns.append(f"link {name} massless (on fixed joint)")
                continue
            errs.append(f"link {name} mass {mass} <= 0 on a movable joint")
        a, b, c = I["ixx"], I["iyy"], I["izz"]
        if min(a, b, c) <= 0 or a + b < c * 0.999 or a + c < b * 0.999 or b + c < a * 0.999:
            warns.append(f"link {name} inertia violates triangle inequality ({a:.2e},{b:.2e},{c:.2e})")
        if l.find("collision") is None and has_geom:
            warns.append(f"link {name} has no collision geometry")
        for cm in l.iter():
            if isinstance(cm.tag, str) or "placeholder inertial" not in (cm.text or ""):
                continue
            warns.append(f"link {name} uses placeholder inertial")

    for j in joints:
        if j.get("type") in ("revolute", "prismatic", "continuous"):
            lim = j.find("limit")
            if lim is None:
                errs.append(f"joint {j.get('name')} has no <limit>")
                continue
            for k in ("effort", "velocity"):
                if float(lim.get(k, "0")) <= 0:
                    errs.append(f"joint {j.get('name')} {k} <= 0")

    sa = standalone_path(path)
    if not sa.exists():
        warns.append("standalone URDF missing (run scripts/build_assets.py)")
    elif sa.stat().st_mtime < path.stat().st_mtime:
        warns.append("standalone URDF older than canonical (run scripts/build_assets.py)")
    else:
        for m in mesh_elements(parse_urdf(sa).getroot()):
            if not (sa.parent / m.get("filename")).is_file():
                errs.append(f"standalone: missing mesh {m.get('filename')}")
    return errs, warns


def check_mujoco(name: str) -> tuple[str, str]:
    try:
        import mujoco
    except ImportError:
        return WARN, "mujoco not installed"
    scene = REPO / "robots" / name / "mjcf" / "scene.xml"
    if not scene.exists():
        return FAIL, "no MJCF (run scripts/build_assets.py --mujoco)"
    try:
        m = mujoco.MjModel.from_xml_path(str(scene))
        d = mujoco.MjData(m)
        for _ in range(500):
            mujoco.mj_step(m, d)
        if not all(abs(x) < 1e6 for x in d.qpos):
            return FAIL, "simulation diverged"
        return OK, f"nq={m.nq} nu={m.nu}"
    except Exception as e:  # noqa: BLE001
        return FAIL, str(e).splitlines()[0]


def check_gazebo(urdf: Path) -> tuple[str, str]:
    exe = shutil.which("ign") or shutil.which("gz")
    if exe is None:
        return WARN, "ign/gz not installed"
    r = subprocess.run([exe, "sdf", "-p", str(urdf)], capture_output=True, text=True)
    if r.returncode != 0 or "<model" not in r.stdout:
        msg = (r.stderr or r.stdout).strip().splitlines()
        return FAIL, msg[0] if msg else "sdf conversion failed"
    # links without inertia are silently dropped by the URDF->SDF converter
    n_urdf = len(parse_urdf(urdf).getroot().findall("link"))
    n_sdf = r.stdout.count("<link ")
    note = f"URDF->SDF ok ({n_sdf}/{n_urdf} links; fixed joints are lumped)"
    return OK, note


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robots", nargs="*")
    ap.add_argument("--mujoco", action="store_true")
    ap.add_argument("--gazebo", action="store_true")
    ap.add_argument("--all", action="store_true", help="--mujoco --gazebo")
    ap.add_argument("--markdown", type=Path, help="write status table to this file")
    ap.add_argument("-v", "--verbose", action="store_true", help="print warnings too")
    a = ap.parse_args()
    if a.all:
        a.mujoco = a.gazebo = True

    reg = load_registry()["robots"]
    rows, failed = [], False
    for name in a.robots or reg:
        cfg = reg[name]
        st_errs, st_warns = [], []
        for urdf in robot_urdfs(cfg):
            if not urdf.exists():
                st_errs.append(f"missing {urdf.relative_to(REPO)}")
                continue
            e, w = check_urdf(urdf)
            st_errs += [f"{urdf.name}: {x}" for x in e]
            st_warns += [f"{urdf.name}: {x}" for x in w]
        static = FAIL if st_errs else (WARN if st_warns else OK)
        row = {"robot": name, "URDF": static}
        if a.gazebo:
            row["Gazebo"] = check_gazebo(REPO / cfg["urdf"])
        if a.mujoco:
            row["MuJoCo"] = check_mujoco(name)
        rows.append(row)
        failed |= bool(st_errs) or any(isinstance(v, tuple) and v[0] == FAIL for v in row.values())
        print(f"[{static}] {name}")
        for x in st_errs:
            print(f"    ERROR {x}")
        if a.verbose:
            for x in st_warns:
                print(f"    warn  {x}")
        elif st_warns:
            print(f"    {len(st_warns)} warnings (use -v)")
        for k in ("Gazebo", "MuJoCo"):
            if k in row:
                print(f"    {k}: {row[k][0]} {row[k][1]}")

    if a.markdown:
        cols = ["robot", "URDF"] + [k for k in ("Gazebo", "MuJoCo") if k in rows[0]]
        lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
        for r in rows:
            lines.append("| " + " | ".join(r[c] if isinstance(r[c], str) else f"{r[c][0]} ({r[c][1]})"
                                           for c in cols) + " |")
        a.markdown.write_text("\n".join(lines) + "\n")
        print(f"status table -> {a.markdown}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
