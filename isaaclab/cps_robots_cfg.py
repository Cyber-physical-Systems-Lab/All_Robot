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
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

REPO = Path(__file__).resolve().parent.parent
REGISTRY = yaml.safe_load(open(REPO / "robots.yaml"))["robots"]

# Drive gains per robot (PhysX implicit PD). Tune for your task.
# Robots listed in ACTUATORS get exactly those groups; all others get GAINS applied to every
# actuated joint of their URDF. Mimic joints are always left passive: PhysX enforces them through
# the importer's mimic constraint, and driving them as well makes the solver blow up (NaN).
GAINS = {  # robot: (stiffness, damping)
    "mycobot_280_pi": (20.0, 2.0),
    "mycobot_280_gripper": (20.0, 2.0),
    "mycomb": (20.0, 2.0),
}
ACTUATORS = {
    "nova5": {"arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], stiffness=400.0, damping=40.0)},
    "nova5_2f85": {
        "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], stiffness=400.0, damping=40.0),
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], stiffness=100.0, damping=5.0),
        "gripper_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_knuckle_joint", ".*_inner_finger_joint", "right_outer_knuckle_joint"],
            stiffness=0.0, damping=0.0),
    },
}


def urdf_joints(name: str) -> tuple[list[str], list[str]]:
    """(actuated, mimic) movable joint names of a robot's canonical URDF."""
    root = ET.parse(REPO / REGISTRY[name]["urdf"]).getroot()
    actuated, mimic = [], []
    for j in root.findall("joint"):
        if j.get("type") in ("revolute", "prismatic", "continuous"):
            (mimic if j.find("mimic") is not None else actuated).append(j.get("name"))
    return actuated, mimic


def actuators(name: str) -> dict:
    if name in ACTUATORS:
        return ACTUATORS[name]
    actuated, mimic = urdf_joints(name)
    k, d = GAINS.get(name, (100.0, 10.0))
    groups = {}
    if actuated:
        groups["joints"] = ImplicitActuatorCfg(joint_names_expr=actuated, stiffness=k, damping=d)
    if mimic:
        groups["mimic_passive"] = ImplicitActuatorCfg(joint_names_expr=mimic, stiffness=0.0, damping=0.0)
    return groups


def is_articulated(name: str) -> bool:
    """False for robots without movable joints (e.g. myAGV shell): the importer then writes a
    plain rigid body without ArticulationRootAPI, so spawn it as a USD prop instead."""
    return bool(REGISTRY[name].get("isaac_builtin")) or any(urdf_joints(name))


def usd_path(name: str) -> Path:
    """Regenerated USD if present, else the legacy hand-made one.

    Newer Isaac Lab URDF converters treat the output path as a folder and write
    <folder>/<urdf stem>/<urdf stem>.usda (+ payloads/); older ones write the file itself. Accept both.
    """
    cfg = REGISTRY[name]
    for key in ("usd", "legacy_usd"):
        p = REPO / cfg[key] if cfg.get(key) else None
        if p is None or not p.exists():
            continue
        if p.is_file():
            return p
        hits = sorted((h for h in p.rglob("*.usd*") if "payloads" not in h.parts and h.is_file()),
                      key=lambda h: (len(h.parts), h.stem != name))  # <name>/<urdf-stem>.usda
        if hits:
            return hits[0]
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
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, float(cfg.get("spawn_z", 0.0))),
            joint_pos=dict(cfg.get("init_joints") or {".*": 0.0})),
        actuators=actuators(name),
    )


def __getattr__(attr: str):
    """Lazy per-robot constants, e.g. NOVA5_2F85_CFG, MYAGV_CFG."""
    if attr.endswith("_CFG") and attr[:-4].lower() in REGISTRY:
        return make_cfg(attr[:-4].lower())
    raise AttributeError(attr)


CPS_ROBOTS = list(REGISTRY)
