#!/usr/bin/env python3
"""Render docs/screenshots/mujoco/<robot>.png offscreen (no window needed).

  python scripts/screenshot_mujoco.py              # all robots with an MJCF
  python scripts/screenshot_mujoco.py nova5 go2
  MUJOCO_GL=egl python scripts/screenshot_mujoco.py   # GPU headless; MUJOCO_GL=osmesa for CPU-only / CI

Needs: env_mujoco or docker/run.sh mujoco, and scripts/build_assets.py --mujoco run first.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (CAM_AZIMUTH_DEG, CAM_ELEVATION_DEG, REPO, SCREENSHOT_DIR,  # noqa: E402
                    camera_view, load_registry, write_png)


def shoot(name: str, cfg: dict, width: int, height: int, settle_steps: int) -> Path:
    import mujoco

    scene = REPO / "robots" / name / "mjcf" / "scene.xml"
    m = mujoco.MjModel.from_xml_path(str(scene))
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, width)
    m.vis.global_.offheight = max(m.vis.global_.offheight, height)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(settle_steps):        # let free-floating robots land on the floor
        mujoco.mj_step(m, d)

    eye, target = camera_view(cfg)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = target
    cam.distance = math.dist(eye, target)
    # MuJoCo azimuth/elevation describe the viewing direction (camera -> target)
    cam.azimuth = CAM_AZIMUTH_DEG - 180.0
    cam.elevation = -CAM_ELEVATION_DEG

    with mujoco.Renderer(m, height=height, width=width) as r:
        r.update_scene(d, camera=cam)
        img = r.render()
    out = SCREENSHOT_DIR / "mujoco" / f"{name}.png"
    write_png(out, img)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robots", nargs="*")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--settle", type=int, default=300, help="sim steps before the shot")
    a = ap.parse_args()
    reg = load_registry()["robots"]
    failed = []
    for name in a.robots or reg:
        if not (REPO / "robots" / name / "mjcf" / "scene.xml").exists():
            print(f"[skip] {name}: no MJCF (python scripts/build_assets.py --mujoco {name})")
            continue
        try:
            print(f"[ok]   {shoot(name, reg[name], a.width, a.height, a.settle).relative_to(REPO)}")
        except Exception as e:  # noqa: BLE001
            failed.append(name)
            print(f"[fail] {name}: {e}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
