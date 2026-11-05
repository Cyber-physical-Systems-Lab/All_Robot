#!/usr/bin/env python3
"""Generate simulator-specific files from the canonical URDFs.

  python3 scripts/build_assets.py                 # standalone URDFs (Isaac Sim, any non-ROS tool)
  python3 scripts/build_assets.py --mujoco        # + MuJoCo meshes (OBJ/STL) and MJCF scenes
  python3 scripts/build_assets.py --mujoco nova5  # only selected robots
  python3 scripts/build_assets.py --isaac         # + Isaac Sim URDFs (flattened meshes where needed)

Outputs (all committed to git so users can skip this step):
  robots/<r>/urdf/standalone/<file>.urdf   mesh paths relative to the file  -> Isaac Sim / Isaac Lab
  robots/<r>/meshes/flat/*.stl             DAE flattened to binary STL: MuJoCo (cannot read DAE), Gazebo
                                           collisions (gazebo.launch.py) and Isaac (DAEs using <instance_node>)
  robots/<r>/urdf/isaac/<file>.urdf        standalone URDF for isaaclab/convert_all.sh
  robots/<r>/mjcf/<r>.xml, scene.xml       MuJoCo model + scene with floor and light

MuJoCo / Isaac extras: pip install mujoco pycollada numpy
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


def write_stl(dst: Path, v, f) -> None:
    import numpy as np
    tri = np.asarray(v, dtype=np.float32)[np.asarray(f)]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tri), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    rec["n"], rec["v"] = n, tri
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "wb") as fh:
        fh.write(f"flattened by All_Robot build_assets.py".encode().ljust(80, b" "))
        fh.write(np.uint32(len(tri)).tobytes())
        fh.write(rec.tobytes())


# --------------------------------------------------------------------------- Isaac URDF
def needs_flattening(dae: Path) -> bool:
    """Onshape-style DAEs place geometry through <instance_node>; Isaac Sim's URDF importer drops
    those node transforms and collapses the parts onto the link origin."""
    return dae.suffix.lower() == ".dae" and b"<instance_node" in dae.read_bytes()


def build_isaac(urdf: Path, log: list[str]) -> Path:
    """Standalone URDF for Isaac Sim with <instance_node> DAEs swapped for flattened OBJs."""
    tree = parse_urdf(urdf)
    out = urdf.parent / "isaac" / urdf.name
    for m in mesh_elements(tree.getroot()):
        p = resolve_uri(m.attrib["filename"])
        if p is not None and needs_flattening(p):
            flat = flat_mesh(p, log)
            log.append(f"{urdf.name}: {p.name} -> {flat.relative_to(REPO)}")
            p = flat
        if p is not None:
            m.attrib["filename"] = relpath(p, out.parent)
    write_urdf(tree, out)
    return out


# --------------------------------------------------------------------------- MuJoCo meshes
def dae_to_stl(src: Path, dst: Path) -> str:
    """Convert a Collada file to one binary STL, applying <unit> scale and node transforms (like RViz)."""
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
            if hasattr(prim, "triangleset"):  # polylist / polygons -> triangles
                prim = prim.triangleset()
            if not hasattr(prim, "vertex_index"):
                continue  # lines etc.
            if prim.vertex is None or len(prim.vertex_index) == 0:
                continue
            verts.append(np.asarray(prim.vertex, dtype=float))
            faces.append(np.asarray(prim.vertex_index, dtype=int).reshape(-1, 3) + off)
            off += len(prim.vertex)
    if not verts:  # geometry only reachable via <instance_node> (e.g. Onshape exports)
        for v_, f_ in _walk_xml_nodes(col):
            verts.append(v_)
            faces.append(f_ + off)
            off += len(v_)
    if not verts:
        raise RuntimeError(f"no triangles in {src}")
    # <up_axis> is deliberately NOT applied: RViz and Gazebo use the raw coordinates as the link
    # frame, and the URDF visual origins were tuned against them (e.g. the Y_UP myCobot M5 meshes).
    v = np.vstack(verts) * unit
    f = np.vstack(faces)
    write_stl(dst, v, f)
    ext = v.max(0) - v.min(0)
    return f"{src.name}: {len(v)} verts, {len(f)} tris, unit={unit}, {up}, extent={np.round(ext, 3).tolist()} m"



def _walk_xml_nodes(col):
    """Yield (verts, faces) by walking the visual scene XML ourselves.

    pycollada drops geometry that sits behind <instance_node> references, so resolve
    <node>/<instance_node>/<instance_geometry> by hand and accumulate node transforms.
    """
    import numpy as np

    root = col.xmlnode.getroot()
    ns = {"c": root.tag[1:].split("}")[0] if root.tag.startswith("{") else ""}
    lib_nodes = {n.get("id"): n for n in root.iterfind("c:library_nodes/c:node", ns)}
    geoms = {g.id: g for g in col.geometries}

    def local(el):
        m = np.eye(4)
        for t in el:
            tag = t.tag.split("}")[-1]
            vals = np.array(t.text.split(), dtype=float) if t.text else None
            if tag == "matrix":
                m = m @ vals.reshape(4, 4)
            elif tag == "translate":
                tm = np.eye(4); tm[:3, 3] = vals; m = m @ tm
            elif tag == "scale":
                m = m @ np.diag([*vals, 1.0])
            elif tag == "rotate":
                ax, ang = vals[:3] / np.linalg.norm(vals[:3]), np.radians(vals[3])
                k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                rm = np.eye(4); rm[:3, :3] = np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * k @ k
                m = m @ rm
        return m

    def visit(el, parent):
        m = parent @ local(el)
        for ig in el.iterfind("c:instance_geometry", ns):
            g = geoms.get(ig.get("url", "").lstrip("#"))
            for prim in (g.primitives if g is not None else []):
                if hasattr(prim, "triangleset"):
                    prim = prim.triangleset()
                if not hasattr(prim, "vertex_index") or prim.vertex is None or len(prim.vertex_index) == 0:
                    continue
                v = np.asarray(prim.vertex, dtype=float)
                v = (np.c_[v, np.ones(len(v))] @ m.T)[:, :3]
                yield v, np.asarray(prim.vertex_index, dtype=int).reshape(-1, 3)
        for inst in el.iterfind("c:instance_node", ns):
            ref = lib_nodes.get(inst.get("url", "").lstrip("#"))
            if ref is not None:
                yield from visit(ref, m)
        for child in el.iterfind("c:node", ns):
            yield from visit(child, m)

    url = root.find("c:scene/c:instance_visual_scene", ns).get("url", "").lstrip("#")
    for vs in root.iterfind("c:library_visual_scenes/c:visual_scene", ns):
        if vs.get("id") == url:
            for n in vs.iterfind("c:node", ns):
                yield from visit(n, np.eye(4))

def flat_mesh(src: Path, log: list[str]) -> Path:
    """Mesh file MuJoCo / Isaac / Gazebo-collision can read: STL and OBJ as they are, DAE flattened
    into robots/<r>/meshes/flat/<stem>.stl (one triangle soup, units + node transforms baked in)."""
    suffix = src.suffix.lower()
    if suffix in (".stl", ".obj"):
        return src
    if suffix != ".dae":
        raise RuntimeError(f"unsupported mesh type: {src}")
    dst = src.parent / "flat" / (src.stem + ".stl")
    if not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime:
        log.append(dae_to_stl(src, dst))
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
        m.attrib["filename"] = relpath(flat_mesh(src, log), mj_dir)
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
    for jnt in world.iter("joint"):  # not <equality><joint>, which has no name
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
    ap.add_argument("--isaac", action="store_true")
    a = ap.parse_args()
    reg = load_registry()["robots"]
    names = a.robots or list(reg)
    failed = []
    for name in names:
        for urdf in robot_urdfs(reg[name]):
            if urdf.exists():
                print(f"[standalone] {build_standalone(urdf).relative_to(REPO)}")
    if a.isaac:
        for name in names:
            log = []
            for urdf in robot_urdfs(reg[name]):
                if urdf.exists():
                    print(f"[isaac] {build_isaac(urdf, log).relative_to(REPO)}")
            print("\n".join(f"[isaac]   {line}" for line in log if "->" in line))
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
