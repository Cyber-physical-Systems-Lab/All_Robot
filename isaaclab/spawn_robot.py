"""Spawn a lab robot in Isaac Lab, wiggle its joints, optionally self-check and screenshot it.

  conda activate env_isaaclab
  python isaaclab/spawn_robot.py --robot nova5_2f85                       # interactive
  python isaaclab/spawn_robot.py --robot all --check --headless           # PASS/FAIL table
  python isaaclab/spawn_robot.py --robot all --screenshot --headless      # docs/screenshots/isaac/<r>.png
  (Docker: docker/run.sh isaaclab /workspace/isaaclab/isaaclab.sh -p isaaclab/spawn_robot.py ...)

--check:      2 s of sim per robot, reports NaNs / explosion / missing joints.
--screenshot: renders an RGB camera (same framing as the other simulators) after the
              robot has settled, saves it, and implies --check-style short runs.
"""
import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--robot", default="nova5_2f85", help="robot name from robots.yaml, or 'all'")
parser.add_argument("--check", action="store_true", help="run a short automatic test and exit")
parser.add_argument("--screenshot", action="store_true", help="save docs/screenshots/isaac/<robot>.png")
parser.add_argument("--width", type=int, default=960)
parser.add_argument("--height", type=int, default=720)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.screenshot:
    args.enable_cameras = True  # camera sensors need the RTX renderer, also when headless
app = AppLauncher(args).app

# --- everything below needs the running app ---
import math  # noqa: E402

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "scripts"))
from common import SCREENSHOT_DIR, camera_view, write_png  # noqa: E402
from cps_robots_cfg import CPS_ROBOTS, REGISTRY, make_cfg  # noqa: E402

SHOT_STEP, CHECK_STEPS = 300, 400


def run(name: str, short: bool) -> tuple[bool, str]:
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 200, device=args.device))
    eye, target = camera_view(REGISTRY[name])
    sim.set_camera_view(list(eye), list(target))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2500.0))
    robot = Articulation(make_cfg(name).replace(prim_path="/World/Robot"))
    camera = None
    if args.screenshot:
        camera = Camera(CameraCfg(
            prim_path="/World/ShotCam", update_period=0, height=args.height, width=args.width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, focus_distance=400.0,
                                             horizontal_aperture=20.955, clipping_range=(0.01, 100.0))))
    sim.reset()
    if camera is not None:
        dev = sim.device
        camera.set_world_poses_from_view(torch.tensor([eye], device=dev), torch.tensor([target], device=dev))
    print(f"[{name}] joints ({robot.num_joints}): {robot.joint_names}")
    lo, hi = robot.data.soft_joint_pos_limits[0, :, 0], robot.data.soft_joint_pos_limits[0, :, 1]
    mid, amp = (lo + hi) / 2, torch.clamp((hi - lo) / 4, max=0.5)
    t, steps, shot = 0.0, 0, ""
    while app.is_running():
        # hold the mid pose while settling for the screenshot, then wiggle
        phase = 0.0 if (camera is not None and steps < SHOT_STEP) else math.sin(2 * math.pi * 0.25 * t)
        if robot.num_joints:
            robot.set_joint_position_target((mid + amp * phase).unsqueeze(0))
        robot.write_data_to_sim()
        sim.step()
        dt = sim.get_physics_dt()
        robot.update(dt)
        if camera is not None:
            camera.update(dt)
            if steps == SHOT_STEP:
                out = SCREENSHOT_DIR / "isaac" / f"{name}.png"
                write_png(out, camera.data.output["rgb"][0, ..., :3].cpu().numpy())
                shot = f", screenshot {out.relative_to(HERE.parent)}"
                print(f"[{name}] saved {out}")
        t += dt
        steps += 1
        if short and steps >= max(CHECK_STEPS, SHOT_STEP + 1):
            break
    if not short:
        return True, "interactive"
    pos = robot.data.root_pos_w[0]
    if torch.isnan(robot.data.joint_pos).any() or torch.isnan(pos).any():
        return False, "NaN in state" + shot
    if pos.abs().max() > 5.0:
        return False, f"root drifted to {pos.tolist()}" + shot
    return True, f"{robot.num_joints} joints, root z={pos[2]:.3f}" + shot


short = args.check or args.screenshot
names = CPS_ROBOTS if args.robot == "all" else [args.robot]
results = {}
for n in names:
    try:
        results[n] = run(n, short)
    except Exception as e:  # noqa: BLE001
        results[n] = (False, repr(e))
    sim_utils.SimulationContext.clear_instance()
    import omni.usd  # noqa: E402
    omni.usd.get_context().new_stage()

if short:
    print("\n| robot | Isaac Lab | note |\n|---|---|---|")
    for n, (ok, note) in results.items():
        print(f"| {n} | {'PASS' if ok else 'FAIL'} | {note} |")
app.close()
