"""Helpers shared by the launch files (reads robots.yaml from the installed share dir)."""
from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

PKG = "cps_robot_description"


def share() -> str:
    return get_package_share_directory(PKG)


def registry() -> dict:
    with open(os.path.join(share(), "robots.yaml")) as f:
        return yaml.safe_load(f)["robots"]


def urdf_path(robot: str, variant: str = "") -> str:
    reg = registry()
    if robot not in reg:
        raise RuntimeError(f"unknown robot '{robot}'. Choose from: {', '.join(reg)}")
    rel = reg[robot]["urdf"]
    if variant:
        matches = [v for v in reg[robot].get("variants", []) if variant in os.path.basename(v)]
        if not matches:
            raise RuntimeError(f"no variant matching '{variant}' for {robot}")
        rel = matches[0]
    return os.path.join(share(), rel)


def read_urdf(path: str) -> str:
    """Plain URDF or xacro -> URDF string."""
    if path.endswith(".xacro"):
        import xacro
        return xacro.process_file(path).toxml()
    with open(path) as f:
        return f.read()


def root_link(urdf_xml: str) -> str:
    root = ET.fromstring(urdf_xml)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    links = [l.get("name") for l in root.findall("link")]
    return next(l for l in links if l not in children)


def weld_to_world(urdf_xml: str, z: float = 0.0) -> str:
    """Add a 'world' link + fixed joint so Gazebo keeps an arm bolted down."""
    root = ET.fromstring(urdf_xml)
    if root.find("link[@name='world']") is not None:
        return urdf_xml
    base = root_link(urdf_xml)
    root.insert(0, ET.Element("link", name="world"))
    j = ET.Element("joint", name="world_to_" + base, type="fixed")
    ET.SubElement(j, "parent", link="world")
    ET.SubElement(j, "child", link=base)
    ET.SubElement(j, "origin", xyz=f"0 0 {z}", rpy="0 0 0")
    root.insert(1, j)
    return ET.tostring(root, encoding="unicode")


def add_gz_joint_control(urdf_xml: str, robot: str, init_joints: dict | None = None,
                         p_gain: float = 200.0, d_gain: float = 10.0) -> tuple[str, list[str]]:
    """Make Gazebo hold the robot in its start pose and report joint states.

    Adds a JointPositionController per actuated (revolute/prismatic/continuous, non-mimic) joint,
    listening on  /<robot>/<joint>/cmd_pos  (gz.msgs.Double), with the start pose taken from
    init_joints ({regex: value}, robots.yaml) or 0. Also adds a JointStatePublisher on
    /<robot>/joint_states. Returns (urdf, actuated joint names).
    """
    init_joints = init_joints or {}
    root = ET.fromstring(urdf_xml)
    joints = []
    for j in root.findall("joint"):
        if j.get("type") not in ("revolute", "prismatic", "continuous") or j.find("mimic") is not None:
            continue
        joints.append(j.get("name"))
    for name in joints:
        start = next((float(v) for pat, v in init_joints.items() if re.fullmatch(pat, name)), 0.0)
        lim = root.find(f"joint[@name='{name}']/limit")
        effort = lim.get("effort") if lim is not None and lim.get("effort") else "100"
        g = ET.SubElement(root, "gazebo")
        pl = ET.SubElement(g, "plugin", filename="ignition-gazebo-joint-position-controller-system",
                           name="ignition::gazebo::systems::JointPositionController")
        for tag, val in (("joint_name", name), ("topic", f"/{robot}/{name}/cmd_pos"),
                         ("p_gain", f"{p_gain:g}"), ("d_gain", f"{d_gain:g}"),
                         ("cmd_max", effort), ("cmd_min", f"-{effort}"),
                         ("initial_position", f"{start:g}")):
            ET.SubElement(pl, tag).text = val
    g = ET.SubElement(root, "gazebo")
    pl = ET.SubElement(g, "plugin", filename="ignition-gazebo-joint-state-publisher-system",
                       name="ignition::gazebo::systems::JointStatePublisher")
    ET.SubElement(pl, "topic").text = f"/{robot}/joint_states"
    return ET.tostring(root, encoding="unicode"), joints


def flatten_collision_meshes(urdf_xml: str) -> str:
    """Point every Collada <collision> mesh at its flattened OBJ copy when one exists
    (robots/<r>/meshes/flat/<stem>.stl, made by scripts/build_assets.py --mujoco).

    Gazebo's DART/ODE collision loader is fragile with Collada: it drops geometry behind
    <instance_node> (ODE "assertion vertices failed"); it also segfaults on OBJ, so STL it is.
    Those STLs are single triangle meshes with units and node transforms baked in, i.e. the same
    shape. Visuals render fine and stay DAE (keeps the materials).
    """
    prefix = f"package://{PKG}/"
    root = ET.fromstring(urdf_xml)
    for m in root.iterfind("link/collision/geometry/mesh"):
        uri = m.get("filename", "")
        if not (uri.startswith(prefix) and uri.lower().endswith(".dae")):
            continue
        dae = os.path.join(share(), uri[len(prefix):])
        flat = os.path.join(os.path.dirname(dae), "flat", os.path.splitext(os.path.basename(dae))[0] + ".stl")
        if os.path.exists(flat):
            m.set("filename", prefix + os.path.relpath(flat, share()))
    return ET.tostring(root, encoding="unicode")


def init_joint_positions(urdf_xml: str, init_joints: dict | None) -> dict:
    """{joint: value} for joint_state_publisher's 'zeros' parameter from robots.yaml init_joints."""
    out = {}
    for j in ET.fromstring(urdf_xml).findall("joint"):
        name = j.get("name")
        if j.get("type") in ("revolute", "prismatic", "continuous") and j.find("mimic") is None:
            for pat, val in (init_joints or {}).items():
                if re.fullmatch(pat, name):
                    out[name] = float(val)
                    break
    return out
