#!/usr/bin/env python3
"""One-time migration: legacy All_Robot + Robots_IsaacLab  ->  this unified layout.

Run from the root of the NEW All_Robot checkout:

  python3 scripts/migrate.py --all-robot ../All_Robot_legacy --isaaclab ../Robots_IsaacLab

  --dry-run   print the plan without copying

Copies meshes/URDFs/USDs into robots/<name>/..., de-duplicates identical files
(sha256), never overwrites a different file silently (prints CONFLICT), then runs
normalize_urdf.py and writes docs/migration_report.md.
"""
from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import REPO  # noqa: E402

# (source-root key, glob relative to that root, destination dir relative to REPO, rename or None)
# A = <All_Robot>/robot_description     B = <Robots_IsaacLab>/robot_descriptions
# Order matters: All_Robot (newer, RViz-verified) wins over Robots_IsaacLab on name clashes.
PLAN = [
    # ---------------- meshes ----------------
    ("A", "meshes/go2/*",                 "robots/go2/meshes", None),
    ("A", "meshes/myagv/*",               "robots/myagv/meshes", None),
    ("B", "meshes/myAGV/*",               "robots/myagv/meshes", None),
    ("A", "meshes/mycobot_280_pi/*",      "robots/mycobot_280_pi/meshes", None),
    ("B", "meshes/myCobot280_pi/*.dae",   "robots/mycobot_280_pi/meshes", None),
    ("B", "meshes/myCobot280_pi/*.png",   "robots/mycobot_280_pi/meshes", None),
    ("B", "meshes/myCobot280_pi/materials/*", "robots/mycobot_280_pi/meshes/materials", None),
    ("B", "meshes/myCobot280_gripper/*.dae", "robots/mycobot_280_gripper/meshes", None),
    ("B", "meshes/myCobot280_gripper/*.png", "robots/mycobot_280_gripper/meshes", None),
    ("A", "meshes/nova5/*",               "robots/nova5/meshes", None),
    ("B", "meshes/nova5/*",               "robots/nova5/meshes", None),
    ("A", "meshes/robotiq_2f85/*",        "robots/robotiq_2f85/meshes", None),
    ("B", "meshes/robotiq_2f85/*",        "robots/robotiq_2f85/meshes", None),
    # ---------------- URDF / xacro ----------------
    ("A", "urdf/go2.urdf",                "robots/go2/urdf", None),
    ("A", "urdf/myagv.urdf",              "robots/myagv/urdf", None),
    ("B", "urdf/myAGV.urdf",              "robots/myagv/urdf/legacy", "myagv_isaaclab.urdf"),
    ("A", "urdf/mycobot_280_pi_with_camera_flange_pump.urdf", "robots/mycobot_280_pi/urdf", None),
    ("B", "urdf/mycobot_280_pi.urdf",     "robots/mycobot_280_pi/urdf", None),
    ("B", "urdf/mycobot_280_pi_with_camera_flange.urdf", "robots/mycobot_280_pi/urdf", None),
    ("B", "urdf/mycobot_280_pi_with_pump.urdf", "robots/mycobot_280_pi/urdf", None),
    ("B", "urdf/mycobot_with_gripper_up.urdf", "robots/mycobot_280_gripper/urdf", "mycobot_280_gripper.urdf"),
    ("A", "urdf/mycomb.urdf",             "robots/mycomb/urdf", None),
    ("A", "urdf/mycomb1.urdf",            "robots/mycomb/urdf", None),
    ("B", "urdf/nova5_robot.urdf",        "robots/nova5/urdf", "nova5.urdf"),
    ("A", "xacro/nova5_robot.xacro",      "robots/nova5/xacro", None),
    ("A", "urdf/nova5_robot_2F85.urdf",   "robots/nova5_2f85/urdf", "nova5_2f85.urdf"),
    ("B", "urdf/nova5_robot_2f85.urdf",   "robots/nova5_2f85/urdf/legacy", "nova5_2f85_isaaclab.urdf"),
    ("B", "urdf/robotiq_2f85.URDF",       "robots/robotiq_2f85/urdf", "robotiq_2f85.urdf"),
    # ---------------- Isaac USD (kept as legacy; regenerate with isaaclab/convert_all.sh) --------
    ("B", "IsaacUSD/myAGV/*",             "robots/myagv/usd/legacy", None),
    ("B", "IsaacUSD/nova5_robot/*",       "robots/nova5/usd/legacy", None),
    ("B", "IsaacUSD/nova5_robot_hand.usd", "robots/nova5/usd/legacy/extra", None),
    ("B", "IsaacUSD/nova5_robot_2f85/*",  "robots/nova5_2f85/usd/legacy", None),
    ("B", "meshes/myCobot280_pi/mycobot_280_pi.usd", "robots/mycobot_280_pi/usd/legacy", None),
    ("B", "meshes/myCobot280_gripper/mycobot_step.usd", "robots/mycobot_280_gripper/usd/legacy", None),
    # ---------------- misc ----------------
    ("A", "rviz/urdf.rviz",               "rviz", None),
]


def sha(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--all-robot", type=Path, required=True, help="clone of the OLD All_Robot repo")
    ap.add_argument("--isaaclab", type=Path, required=True, help="clone of Robots_IsaacLab")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    roots = {"A": a.all_robot / "robot_description", "B": a.isaaclab / "robot_descriptions"}
    for k, r in roots.items():
        if not r.is_dir():
            sys.exit(f"[migrate] not found: {r}")

    log = ["# Migration report", "", "| action | source | destination |", "|---|---|---|"]
    copied = dup = conflicts = 0
    for key, pattern, dest, rename in PLAN:
        srcs = sorted(p for p in roots[key].glob(pattern) if p.is_file())
        if not srcs:
            print(f"[migrate] WARNING no match for {key}:{pattern}")
            log.append(f"| missing | {key}:{pattern} | |")
        for src in srcs:
            dst = REPO / dest / (rename or src.name)
            rel_src = f"{'All_Robot' if key == 'A' else 'Robots_IsaacLab'}/{src.relative_to(roots[key].parent)}"
            if dst.exists():
                if sha(dst) == sha(src):
                    dup += 1
                    continue
                conflicts += 1
                print(f"[migrate] CONFLICT kept existing {dst.relative_to(REPO)} (differs from {rel_src})")
                log.append(f"| conflict (kept first) | {rel_src} | {dst.relative_to(REPO)} |")
                continue
            log.append(f"| copy | {rel_src} | {dst.relative_to(REPO)} |")
            copied += 1
            if not a.dry_run:
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)

    # keep the README images from the old All_Robot
    for img in ("doge.gif", "robots_in_simulation.png"):
        src, dst = a.all_robot / "docs" / img, REPO / "docs" / img
        if src.exists() and not dst.exists() and not a.dry_run:
            dst.parent.mkdir(exist_ok=True)
            shutil.copy2(src, dst)

    print(f"[migrate] copied={copied} identical-duplicates-skipped={dup} conflicts={conflicts}")
    if a.dry_run:
        print("\n".join(log))
        return 0

    (REPO / "docs").mkdir(exist_ok=True)
    (REPO / "docs" / "migration_report.md").write_text("\n".join(log) + "\n")
    print("[migrate] normalising URDFs ...", flush=True)
    rc = subprocess.call([sys.executable, str(REPO / "scripts" / "normalize_urdf.py"),
                          "--report", str(REPO / "docs" / "normalize_report.md")])
    print("[migrate] done. Next: python3 scripts/build_assets.py && python3 scripts/validate.py")
    return rc


if __name__ == "__main__":
    sys.exit(main())
