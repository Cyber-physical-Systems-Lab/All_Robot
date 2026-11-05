"""Helpers shared by the launch files (reads robots.yaml from the installed share dir)."""
import os
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
