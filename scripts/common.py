"""Shared helpers for the All_Robot tooling (no ROS / simulator dependency)."""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
XACRO_NS = "http://www.ros.org/wiki/xacro"


def load_registry() -> dict:
    with open(REPO / "robots.yaml") as f:
        return yaml.safe_load(f)


def package_name() -> str:
    return load_registry()["package"]


def robot_urdfs(cfg: dict) -> list[Path]:
    """Canonical URDF + all variants of one robot entry."""
    paths = [cfg["urdf"], *cfg.get("variants", [])]
    return [REPO / p for p in paths]


def standalone_path(urdf: Path) -> Path:
    """robots/x/urdf/foo.urdf -> robots/x/urdf/standalone/foo.urdf (relative mesh paths)."""
    return urdf.parent / "standalone" / urdf.name


def parse_urdf(path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser)


def write_urdf(tree: ET.ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.register_namespace("xacro", XACRO_NS)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    with open(path, "a") as f:
        f.write("\n")


def resolve_uri(uri: str, base_dir: Path | None = None) -> Path | None:
    """Resolve a URDF mesh URI to a file inside this repo.

    package://<pkg>/<rel>  -> REPO/<rel>   (pkg must be ours)
    file://<abs>           -> abs
    relative path          -> base_dir/<rel>
    """
    pkg = package_name()
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        name, _, rel = rest.partition("/")
        if name != pkg:
            return None
        return REPO / rel
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    p = Path(uri)
    if p.is_absolute():
        return p
    return (base_dir or REPO) / p


def to_package_uri(path: Path) -> str:
    return f"package://{package_name()}/{path.resolve().relative_to(REPO).as_posix()}"


def mesh_elements(root: ET.Element):
    for m in root.iter("mesh"):
        if "filename" in m.attrib:
            yield m


def relpath(target: Path, start: Path) -> str:
    return Path(os.path.relpath(target, start)).as_posix()


# --------------------------------------------------------------------------- screenshots
SCREENSHOT_DIR = REPO / "docs" / "screenshots"
SIMULATORS = {  # folder name -> column title in the README gallery
    "rviz": "RViz2",
    "gazebo": "Gazebo Fortress",
    "mujoco": "MuJoCo",
    "isaac": "Isaac Sim / Lab",
}
CAM_AZIMUTH_DEG = 45.0     # camera sits on the +x+y diagonal ...
CAM_ELEVATION_DEG = 25.0   # ... 25 deg above the target -> identical framing in every simulator


def camera_view(cfg: dict) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(eye, target) in world coordinates for a robot entry of robots.yaml."""
    import math
    cam = cfg.get("camera", {})
    d, tz = float(cam.get("distance", 1.5)), float(cam.get("target_z", 0.3))
    az, el = math.radians(CAM_AZIMUTH_DEG), math.radians(CAM_ELEVATION_DEG)
    eye = (d * math.cos(el) * math.cos(az), d * math.cos(el) * math.sin(az), tz + d * math.sin(el))
    return eye, (0.0, 0.0, tz)


def write_png(path: Path, rgb) -> None:
    """Write an HxWx3 uint8 array as PNG with only numpy + zlib (no Pillow needed)."""
    import struct
    import zlib

    import numpy as np
    a = np.ascontiguousarray(np.asarray(rgb, dtype=np.uint8)[..., :3])
    h, w, _ = a.shape
    raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
