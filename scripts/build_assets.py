#!/usr/bin/env python3
"""Generate simulator-specific files from the canonical URDFs.

  python3 scripts/build_assets.py                 # standalone URDFs (Isaac Sim, any non-ROS tool)
  python3 scripts/build_assets.py --mujoco        # + MuJoCo meshes (OBJ/STL) and MJCF scenes
  python3 scripts/build_assets.py --mujoco nova5  # only selected robots

Outputs (all committed to git so users can skip this step):
  robots/<r>/urdf/standalone/<file>.urdf   mesh paths relative to the file  -> Isaac Sim / Isaac Lab
  robots/<r>/meshes/mujoco/*.obj|*.stl     DAE converted to OBJ (MuJoCo cannot read DAE)
  robots/<r>/mjcf/<r>.xml, scene.xml       MuJoCo model + scene with floor and light

MuJoCo extras: pip install mujoco pycollada numpy
"""
from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (REPO, load_registry, mesh_elements, parse_urdf, relpath,  # noqa: E402
                    resolve_uri, robot_urdfs, standalone_path, write_urdf)


# --------------------------------------------------------------------------- standalone URDF
def build_standalone(urdf: Path) -> Path:
    tree = parse_urdf(urdf)
    out = standalone_path(urdf)
    for m in mesh_elements(tree.getroot()):
        p = resolve_uri(m.attrib["filename"])
        if p is not None:
            m.attrib["filename"] = relpath(p, out.parent)
    write_urdf(tree, out)
    return out


# --------------------------------------------------------------------------- MuJoCo meshes
def dae_to_obj(src: Path, dst: Path) -> str:
    """Convert a Collada file to a single OBJ, applying <unit> scale, node transforms and up-axis."""
    import collada
    import numpy as np

    col = collada.Collada(str(src), ignore=[collada.common.DaeUnsupportedError,
                                            collada.common.DaeBrokenRefError])
    unit = float(getattr(col.assetInfo, "unitmeter", None) or 1.0)
    up = str(getattr(col.assetInfo, "upaxis", "Z_UP"))
    verts, faces, off = [], [], 0
    geoms = col.scene.objects("geometry") if col.scene is not None else []
    for g in geoms:
        for prim in g.primitives():
            if not hasattr(prim, "vertex_index"):
                if hasattr(prim, "triangleset"):
                    prim = prim.triangleset()
                else:
                    continue  # lines etc.
            if prim.vertex is None or len(prim.vertex_index) == 0:
                continue
            verts.append(np.asarray(prim.vertex, dtype=float))
            faces.append(np.asarray(prim.vertex_index, dtype=int).reshape(-1, 3) + off)
            off += len(prim.vertex)
    if not verts:
        raise RuntimeError(f"no triangles in {src}")
    v = np.vstack(verts) * unit
    if "Y_UP" in up:  # Collada Y-up -> ROS Z-up
        v = v[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    f = np.vstack(faces)
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "w") as fh:
        fh.write(f"# converted from {src.name} unit={unit} up={up}\n")
        np.savetxt(fh, v, fmt="v %.6f %.6f %.6f")
        np.savetxt(fh, f + 1, fmt="f %d %d %d")
    ext = v.max(0) - v.min(0)
    return f"{src.name}: {len(v)} verts, {len(f)} tris, unit={unit}, {up}, extent={np.round(ext, 3).tolist()} m"


def mujoco_mesh(src: Path, log: list[str]) -> Path:
    """Return the MuJoCo-readable copy of a mesh, converting if needed."""
    out_dir = src.parent / "mujoco"
    suffix = src.suffix.lower()
    if suffix == ".dae":
        dst = out_dir / (src.stem + ".obj")
        if not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime:
            log.append(dae_to_obj(src, dst))
    elif suffix in (".stl", ".obj"):
        dst = out_dir / (src.stem + suffix)
        if not dst.exists():
            out_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    else:
        raise RuntimeError(f"unsupported mesh type for MuJoCo: {src}")
    return dst


# --------------------------------------------------------------------------- MJCF
def build_mjcf(name: str, cfg: dict, log: list[str]) -> Path:
    import mujoco

    urdf = REPO / cfg["urdf"]
    mj_dir = REPO / "robots" / name / "mjcf"
    mj_dir.mkdir(parents=True, exist_ok=True)
    tree = parse_urdf(urdf)
    root = tree.getroot()

    mimic = []
    for j in root.findall("joint"):
        mm = j.find("mimic")
        if mm is not None:
            mimic.append((j.get("name"), mm.get("joint"),
                          float(mm.get("multiplier", 1)), float(mm.get("offset", 0))))
            j.remove(mm)
    for m in mesh_elements(root):
        src = resolve_uri(m.attrib["filename"])
        m.attrib["filename"] = relpath(mujoco_mesh(src, log), mj_dir)
    ext = ET.SubElement(root, "mujoco")
    ET.SubElement(ext, "compiler", strippath="false", discardvisual="false",
                  fusestatic="false", balanceinertia="true", angle="radian")

    tmp = mj_dir / f"_import_{name}.urdf"
    tree.write(tmp, encoding="utf-8", xml_declaration=True)
    try:
        model = mujoco.MjModel.from_xml_path(str(tmp))
        out = mj_dir / f"{name}.xml"
        mujoco.mj_saveLastXML(str(out), model)
    finally:
        tmp.unlink(missing_ok=True)

    # ---- post-process: free base, mimic equalities, position actuators
    mj = ET.parse(out)
    mroot = mj.getroot()
    world = mroot.find("worldbody")
    if not cfg.get("fixed_base", True) and world.find("body[@name='floating_base']") is None:
        # Wrap EVERYTHING under worldbody (the root link may be imported as world geoms or as a
        # body, depending on the MuJoCo version) into one free body.
        free = ET.Element("body", name="floating_base", pos=f"0 0 {cfg.get('spawn_z', 0.0)}")
        free.append(ET.Element("freejoint", name="root"))
        free.append(ET.Element("inertial", pos="0 0 0", mass="0.001", diaginertia="1e-6 1e-6 1e-6"))
        for child in list(world):
            if child.tag in ("body", "geom", "site", "frame"):
                world.remove(child)
                free.append(child)
        world.append(free)
    if mimic:
        eq = mroot.find("equality")
        if eq is None:
            eq = ET.SubElement(mroot, "equality")
        for child, parent, mult, offset in mimic:
            ET.SubElement(eq, "joint", joint1=child, joint2=parent,
                          polycoef=f"{offset:g} {mult:g} 0 0 0", solimp="0.95 0.99 0.001")
    mimic_children = {c for c, *_ in mimic}
    act = mroot.find("actuator")
    if act is None:
        act = ET.SubElement(mroot, "actuator")
    kp = float(cfg.get("mujoco_kp", 50.0 if cfg.get("fixed_base", True) else 20.0))
    for jnt in mroot.iter("joint"):
        if jnt.get("type", "hinge") not in ("hinge", "slide") or jnt.get("name") in mimic_children:
            continue
        jnt.set("damping", jnt.get("damping", "1"))
        jnt.set("armature", jnt.get("armature", "0.01"))
        a = ET.SubElement(act, "position", name=f"act_{jnt.get('name')}",
                          joint=jnt.get("name"), kp=f"{kp:g}")
        if jnt.get("range"):
            a.set("ctrlrange", jnt.get("range"))
    if len(act) == 0:
        mroot.remove(act)
    ET.indent(mj, space="  ")
    mj.write(out, encoding="utf-8")

    scene = mj_dir / "scene.xml"
    scene.write_text(f"""<mujoco model="{name} scene">
  <include file="{name}.xml"/>
  <statistic center="0 0 0.3" extent="1.2"/>
  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
""")
    # sanity: compile the scene and step it
    m = mujoco.MjModel.from_xml_path(str(scene))
    d = mujoco.MjData(m)
    for _ in range(200):
        mujoco.mj_step(m, d)
    log.append(f"{name}: MJCF ok (nbody={m.nbody}, njnt={m.njnt}, nu={m.nu}, neq={m.neq})")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robots", nargs="*")
    ap.add_argument("--mujoco", action="store_true")
    a = ap.parse_args()
    reg = load_registry()["robots"]
    names = a.robots or list(reg)
    failed = []
    for name in names:
        for urdf in robot_urdfs(reg[name]):
            if urdf.exists():
                print(f"[standalone] {build_standalone(urdf).relative_to(REPO)}")
    if a.mujoco:
        for name in names:
            log: list[str] = []
            try:
                build_mjcf(name, reg[name], log)
            except Exception as e:  # keep going, report at the end
                failed.append(name)
                log.append(f"{name}: MJCF FAILED: {e}")
            print("\n".join(f"[mujoco] {line}" for line in log))
    if failed:
        print(f"[mujoco] failed: {failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
