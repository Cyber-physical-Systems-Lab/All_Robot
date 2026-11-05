"""Isaac Lab 2.x ArticulationCfg for every lab robot.

Usage inside your own Isaac Lab project:

    import sys; sys.path.append("/path/to/All_Robot/isaaclab")
    from cps_robots_cfg import CPS_ROBOTS, NOVA5_2F85_CFG
    robot_cfg = NOVA5_2F85_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

USD resolution order per robot: robots/<r>/usd/<r>.usd (regenerated, preferred)
-> legacy_usd from robots.yaml. Go2 uses Isaac Lab's built-in UNITREE_GO2_CFG.
"""
from __future__ import annotations

import importlib
from pathlib import Path

import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

REPO = Path(__file__).resolve().parent.parent
REGISTRY = yaml.safe_load(open(REPO / "robots.yaml"))["robots"]

# Drive gains per robot (PhysX implicit PD). Tune for your task.
ACTUATORS = {
    "nova5": {"arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], stiffness=400.0, damping=40.0)},
    "nova5_2f85": {
        "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], stiffness=400.0, damping=40.0),
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], stiffness=100.0, damping=5.0),
        # mimic joints are converted to PhysX mimic/tendon joints by the importer; keep them passive
        "gripper_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_knuckle_joint", ".*_inner_finger_joint", "right_outer_knuckle_joint"],
            stiffness=0.0, damping=0.0),
    },
    "robotiq_2f85": {"gripper": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=100.0, damping=5.0)},
    "mycobot_280_pi": {"arm": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=20.0, damping=2.0)},
    "mycobot_280_gripper": {"arm": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=20.0, damping=2.0)},
    "mycomb": {"arm": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=20.0, damping=2.0)},
    "myagv": {},
}


def usd_path(name: str) -> Path:
    cfg = REGISTRY[name]
    for key in ("usd", "legacy_usd"):
        if cfg.get(key) and (REPO / cfg[key]).exists():
            return REPO / cfg[key]
    raise FileNotFoundError(f"No USD for {name}: run isaaclab/convert_all.sh {name}")


def make_cfg(name: str) -> ArticulationCfg:
    cfg = REGISTRY[name]
    if cfg.get("isaac_builtin"):
        mod, _, attr = cfg["isaac_builtin"].rpartition(".")
        return getattr(importlib.import_module(mod), attr)
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path(name)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8,
                solver_velocity_iteration_count=0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, float(cfg.get("spawn_z", 0.0)))),
        actuators=ACTUATORS.get(name, {}),
    )


def __getattr__(attr: str):
    """Lazy per-robot constants, e.g. NOVA5_2F85_CFG, MYAGV_CFG."""
    if attr.endswith("_CFG") and attr[:-4].lower() in REGISTRY:
        return make_cfg(attr[:-4].lower())
    raise AttributeError(attr)


CPS_ROBOTS = list(REGISTRY)
