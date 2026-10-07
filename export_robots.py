"""Export the robot showcase into one compiled MuJoCo model for the headset.

    python export_robots.py      ->  scenes/robots.mjb   (needs robosuite, robosuite_models, robocasa + its assets)

ROBOTS lists each robot once: robosuite robots are built exactly as RoboCasa builds them (robot,
default gripper, default base, RoboCasa height offset, RoboCasa home pose); menagerie robots are
attached from mujoco_menagerie (home keyframe if they have one; floating bases are fixed at
their home pose). They stand on an arc around the user's start position, facing the user.

Collision geoms are kept (moved to group 3, hidden) so the headset can simulate contacts;
textures are dropped and meshes simplified per robot (cached in scenes/meshcache). Custom texts
in the model: "robots" maps display names to top-level bodies (menu show/hide), "effectors"
lists the bodies the user can grab and drive with IK.
The .mjb must be loaded by the same MuJoCo version that wrote it (3.3.1 here and on Android).
"""
import hashlib
import os
import xml.etree.ElementTree as ET

import fast_simplification
import mujoco
import numpy as np
from PIL import Image
import robosuite_models  # noqa: F401  registers G1
import trimesh
from robosuite.controllers import load_composite_controller_config
from robosuite.models.world import MujocoWorldBase
from robosuite.robots import ROBOT_CLASS_MAPPING

import robocasa
from robocasa.utils.env_utils import _ROBOT_POS_OFFSETS

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "scenes", "meshcache")
MENAGERIE = os.path.join(HERE, "menagerie")

# (display name, source, robosuite robot name or menagerie xml, mesh simplification ratio,
#  menagerie end-effector bodies/"site:" names; robosuite robots use their grippers' grip sites)
# RoboCasa robots: PandaOmron, GR1 (also FloatingBody/FixedLowerBody variants, same look), G1 (same),
# GoogleRobot (not in robosuite; menagerie's google_robot is that robot). Ratios keep the whole
# scene around 400k visible triangles for 72 fps on Quest.
ROBOTS = [
    ("Franka Panda", "menagerie", "franka_emika_panda/panda.xml", 0.5, ["hand"]),
    ("UR5e", "menagerie", "universal_robots_ur5e/ur5e.xml", 0.2, ["wrist_3_link"]),
    ("Kinova Gen3", "menagerie", "kinova_gen3/gen3.xml", 0.25, ["bracelet_link"]),
    ("PandaOmron", "robosuite", "PandaOmron", 0.1, []),
    ("ALOHA", "menagerie", "aloha/aloha.xml", 0.12, ["site:left/gripper", "site:right/gripper"]),
    ("GR1", "robosuite", "GR1", 0.1, []),
    ("G1", "robosuite", "G1", 0.1, []),
    ("Unitree H1", "menagerie", "unitree_h1/h1.xml", 0.12, ["left_elbow_link", "right_elbow_link"]),
    ("Spot", "menagerie", "boston_dynamics_spot/spot_arm.xml", 0.2, ["arm_link_wr1"]),
    ("Stretch 3", "menagerie", "hello_robot_stretch_3/stretch.xml", 0.12, ["link_SG3_gripper_body"]),
    ("Google Robot", "menagerie", "google_robot/robot.xml", 1.0, ["site:gripper"]),
]
# RoboCasa items (robots RoboCasa uses + the table with RoboCasa objects): one menu toggle shows/hides all
ROBOCASA = ["PandaOmron", "GR1", "G1", "Google Robot", "Table"]
# table in the middle of the arc, in front of the user, with RoboCasa objaverse objects on it
TABLE_POS = (-1.6, 0.0)  # MuJoCo x, y (user starts at the origin looking along -x)
TABLE_TOP = 0.75  # m
TABLE_SIZE = (0.4, 0.75)  # half extents x, y (m)
OBJ_DIR = os.path.join(os.path.dirname(robocasa.__file__), "models", "assets", "objects", "objaverse")
TABLE_OBJECTS = ["apple/apple_0", "mug/mug_0", "bowl/bowl_0", "banana/banana_1",
                 "bottled_water/bottled_water_1", "croissant/croissant_0", "can/can_0"]
ARC_RADIUS = 4.0  # m from the user's start position
ARC_SPAN = 220  # degrees covered by the arc, centred straight ahead
MESH_MIN_TRIS = 300  # never simplify a mesh below this
HULL_MAX_FACES = 120  # collision-only meshes become convex hulls this small (MuJoCo collides hulls anyway)
FLOOR_HALF = 50  # m: big enough to fade into the horizon haze


def simplify(path, ratio):
    """Simplified copy of a mesh file (binary STL in the cache), or the original if small enough."""
    if ratio >= 1:
        return path
    os.makedirs(CACHE, exist_ok=True)
    out = os.path.join(CACHE, hashlib.md5(f"{path}{ratio}{MESH_MIN_TRIS}".encode()).hexdigest() + ".stl")
    if os.path.exists(out):
        return out
    tm = trimesh.load(path, force="mesh")
    # positions + faces only: drops UV seams so shared corners merge into one connected surface
    # (STL stores each triangle separately; OBJ splits vertices at texture seams). Unmerged
    # triangle soup makes the simplifier tear holes.
    tm = trimesh.Trimesh(tm.vertices, tm.faces)
    n = len(tm.faces)
    target = max(MESH_MIN_TRIS, int(n * ratio))
    if n <= target:
        return path
    v, f = fast_simplification.simplify(tm.vertices, tm.faces, target_reduction=1 - target / n)
    trimesh.Trimesh(v, f, process=False).export(out)
    return out


def arc_pose(k, n):
    """Position (x, y) and yaw of robot k of n on the arc; the user starts at the origin looking
    along MuJoCo -x (see scenePlacement in main.cpp), robots face the origin."""
    a = np.radians((k / max(n - 1, 1) - 0.5) * ARC_SPAN)
    x, y = -ARC_RADIUS * np.cos(a), ARC_RADIUS * np.sin(a)
    return x, y, np.arctan2(-y, -x)


def hull(path):
    """Small convex hull of a mesh file (binary STL in the cache) for collision-only meshes."""
    os.makedirs(CACHE, exist_ok=True)
    out = os.path.join(CACHE, hashlib.md5(f"hull{path}{HULL_MAX_FACES}".encode()).hexdigest() + ".stl")
    if not os.path.exists(out):
        h = trimesh.load(path, force="mesh").convex_hull
        if len(h.faces) > HULL_MAX_FACES:
            v, f = fast_simplification.simplify(h.vertices, h.faces, target_reduction=1 - HULL_MAX_FACES / len(h.faces))
            h = trimesh.Trimesh(v, f).convex_hull
        h.export(out)
    return out


def hull_collision_meshes(spec):
    """Replace meshes used only by collision geoms with small convex hulls: cheaper contacts, same look."""
    used = {}
    for g in spec.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_MESH:
            used.setdefault(g.meshname, set()).add(bool(g.contype or g.conaffinity))
    n = 0
    for mesh in spec.meshes:
        name = mesh.name or os.path.splitext(os.path.basename(mesh.file))[0]
        if mesh.file and used.get(name) == {True}:
            mesh.file = hull(mesh.file if os.path.isabs(mesh.file) else os.path.join(spec.meshdir, mesh.file))
            n += 1
    return n


def robosuite_world(entries):
    """robosuite robots merged into one world XML; returns (xml root, {name: robot}, {name: root bodies})."""
    world = MujocoWorldBase()  # no arena: robosuite's has 3 m walls around the origin
    robots, roots = {}, {}
    for i, (name, rs_name, (x, y, yaw), _) in enumerate(entries):
        r = ROBOT_CLASS_MAPPING[rs_name](
            robot_type=rs_name, idn=i, composite_controller_config=load_composite_controller_config(robot=rs_name)
        )
        r.load_model()
        # robosuite names this mobile-base body without the robot prefix: fine for one robot, not a row
        for body in r.robot_model.worldbody.iter("body"):
            if body.get("name") == "manipulator_mount":
                body.set("name", f"robot{i}_manipulator_mount")
        r.robot_model.set_base_xpos([x, y, _ROBOT_POS_OFFSETS.get(rs_name, [0, 0, 0])[2]])
        r.robot_model.set_base_ori([0, 0, yaw])
        before = len(world.worldbody.findall("body"))
        world.merge(r.robot_model)
        roots[name] = [b.get("name") for b in world.worldbody.findall("body")[before:]]
        robots[name] = r
    return ET.fromstring(world.get_xml()), robots, roots


def strip_for_display(root, ratios):
    """Hide collision geoms (group 3, still simulated), drop robot floors and textures, simplify meshes."""
    worldbody = root.find("worldbody")
    for g in worldbody.findall("geom"):
        if g.get("type") == "plane":
            worldbody.remove(g)  # e.g. GR1 files bring their own floor plane
    for body in root.iter("body"):
        geoms = body.findall("geom")
        visual = [g for g in geoms if g.get("contype") == "0" and g.get("conaffinity") == "0"]
        if visual:
            for g in geoms:
                if g not in visual and g.get("type") != "plane":
                    g.set("group", "3")
    asset = root.find("asset")
    used = {g.get("mesh") for g in root.iter("geom")}
    for mesh in asset.findall("mesh"):
        if mesh.get("name") not in used:
            asset.remove(mesh)
        else:  # robosuite mesh names carry the robot index, e.g. robot2_ / gripper2_ / mobilebase2_
            idn = next((int(c) for c in mesh.get("name") if c.isdigit()), 0)
            mesh.set("file", simplify(mesh.get("file"), ratios[idn]))
    for tex in asset.findall("texture"):
        asset.remove(tex)
    for mat in asset.findall("material"):
        mat.attrib.pop("texture", None)


def attach_menagerie(spec, xml, prefix, pose, ratio):
    """Attach a menagerie robot at pose (x, y, yaw); returns its root body names and home qpos by joint name."""
    path = os.path.join(MENAGERIE, xml)
    ref = mujoco.MjModel.from_xml_path(path)
    key = ref.key_qpos[0] if ref.nkey else ref.qpos0
    home = {ref.joint(j).name: key[ref.jnt_qposadr[j]] for j in range(ref.njnt)
            if ref.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE}
    child = mujoco.MjSpec.from_file(path)
    meshdir = os.path.join(os.path.dirname(path), child.meshdir)
    for mesh in child.meshes:
        if mesh.file:
            if not mesh.name:  # unnamed meshes are named after their file; keep that name for the geoms
                mesh.name = os.path.splitext(os.path.basename(mesh.file))[0]
            mesh.file = simplify(os.path.join(meshdir, mesh.file), ratio)
    for t in list(child.textures):
        t.delete()
    for mat in child.materials:
        mat.textures = [""] * len(mat.textures)
    # floating base (Spot, H1, Stretch): fix it where the home pose puts it
    for j in [j for j in child.joints if j.type == mujoco.mjtJoint.mjJNT_FREE]:
        fj = ref.joint(j.name).id if j.name else next(
            i for i in range(ref.njnt) if ref.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE)
        q = key[ref.jnt_qposadr[fj]:ref.jnt_qposadr[fj] + 7]  # position xyz + quaternion
        body = next(b for b in child.bodies if j in b.joints)
        body.pos, body.quat = q[0:3], q[3:7]
        j.delete()
    for k in list(child.keys):  # home pose is baked into qpos0 below; stale keys crash attach after edits
        k.delete()
    x, y, yaw = pose
    frame = spec.worldbody.add_frame(pos=[x, y, 0], quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    roots = [prefix + b.name for b in child.worldbody.bodies]
    spec.attach(child, prefix=prefix, frame=frame)  # whole model: all roots (ALOHA has two), assets once
    return roots, {prefix + j: q for j, q in home.items()}


def texture_color(path):
    """Average colour of a texture image: the headset renderer has no textures, so objects get this."""
    img = np.asarray(Image.open(path).convert("RGB").resize((64, 64)), dtype=float) / 255
    return [*img.reshape(-1, 3).mean(0), 1.0]


def add_table(spec):
    """Box table at TABLE_POS with RoboCasa objects on top (static: welded to the table)."""
    x, y = TABLE_POS
    hx, hy = TABLE_SIZE
    table = spec.worldbody.add_body(name="table", pos=[x, y, 0])
    wood = [0.55, 0.38, 0.24, 1]
    table.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, 0.02], pos=[0, 0, TABLE_TOP - 0.02], rgba=wood)
    for sx in (-1, 1):
        for sy in (-1, 1):
            table.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.025, 0.025, (TABLE_TOP - 0.04) / 2],
                           pos=[sx * (hx - 0.05), sy * (hy - 0.05), (TABLE_TOP - 0.04) / 2], rgba=wood)
    ys = np.linspace(-hy + 0.12, hy - 0.12, len(TABLE_OBJECTS))
    for k, (obj, oy) in enumerate(zip(TABLE_OBJECTS, ys)):
        path = os.path.join(OBJ_DIR, obj, "model.xml")
        child = mujoco.MjSpec.from_file(path)
        for mesh in child.meshes:  # absolute paths: the parent spec has a different mesh directory
            if mesh.file and not os.path.isabs(mesh.file):
                mesh.file = os.path.join(os.path.dirname(path), child.meshdir, mesh.file)
        tex_files = {t.name: os.path.join(os.path.dirname(path), t.file) for t in child.textures}
        for mat in child.materials:
            names = [n for n in mat.textures if n]
            if names and names[0] in tex_files:
                mat.rgba = texture_color(tex_files[names[0]])
            mat.textures = [""] * len(mat.textures)
        for t in list(child.textures):
            t.delete()
        bottom = 0.0
        for g in child.geoms:
            if g.name == "reg_bbox":  # RoboCasa's placement box: sits the object on the table top
                bottom = g.pos[2] - g.size[2]
            elif g.contype or g.conaffinity:
                g.group = 3  # collision pieces: hidden
        frame = table.add_frame(pos=[0, oy, TABLE_TOP - bottom])
        spec.attach(child, prefix=f"obj{k}_", frame=frame)
    return ["table"]


def main():
    poses = [arc_pose(k, len(ROBOTS)) for k in range(len(ROBOTS))]
    rs_entries = [(n, src, pose, r) for (n, kind, src, r, _), pose in zip(ROBOTS, poses) if kind == "robosuite"]
    root, rs_robots, roots = robosuite_world(rs_entries)
    strip_for_display(root, [r for *_, r in rs_entries])
    ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane",
                  size=f"{FLOOR_HALF} {FLOOR_HALF} 0.1", rgba="1 1 1 1")

    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    # a few dummy links have no mass at all; clamp only those (real links keep their real mass)
    spec.compiler.boundmass = 1e-3
    spec.compiler.boundinertia = 1e-6
    homes, effectors = {}, []
    for k, ((name, kind, src, ratio, effs), pose) in enumerate(zip(ROBOTS, poses)):
        if kind == "menagerie":
            roots[name], home = attach_menagerie(spec, src, f"m{k}_", pose, ratio)
            homes.update(home)
            effectors += [f"site:m{k}_{e[5:]}" if e.startswith("site:") else f"m{k}_{e}" for e in effs]
        else:
            r = rs_robots[name]
            effectors += [f"site:{r.gripper[arm].important_sites['grip_site']}" for arm in r.arms]
    roots["Table"] = add_table(spec)
    hulls = hull_collision_meshes(spec)
    # display name -> top-level bodies, in arc order, for the headset menu
    names = [n for n, *_ in ROBOTS] + ["Table"]
    spec.add_text(name="robots", data=";".join(f"{n}={','.join(roots[n])}" for n in names))
    spec.add_text(name="robocasa", data=",".join(ROBOCASA))
    m = spec.compile()
    # grabbable end effectors as body names (sites resolved to their body)
    eff_bodies = [m.body(int(m.site(e[5:]).bodyid[0])).name if e.startswith("site:") else m.body(e).name for e in effectors]
    spec.add_text(name="effectors", data=",".join(eff_bodies))
    m = spec.compile()

    # home poses -> qpos0, so mj_resetData puts every robot in its start pose
    for r in rs_robots.values():
        for jname, q in zip(r.robot_model.joints, r.init_qpos):
            m.qpos0[m.jnt_qposadr[m.joint(jname).id]] = q
    for jname, q in homes.items():
        m.qpos0[m.jnt_qposadr[m.joint(jname).id]] = q

    os.makedirs(os.path.join(HERE, "scenes"), exist_ok=True)
    out = os.path.join(HERE, "scenes", "robots.mjb")
    mujoco.mj_saveModel(m, out, None)
    visible = [i for i in range(m.ngeom) if m.geom_group[i] < 3]
    tris = sum(m.mesh_facenum[m.geom_dataid[i]] for i in visible if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH)
    print(f"{out}: {os.path.getsize(out) / 1e6:.1f} MB, robots: {', '.join(n for n, *_ in ROBOTS)}; "
          f"ngeom={m.ngeom} nmesh={m.nmesh} visible mesh triangles={tris} collision hulls={hulls}\n"
          f"effectors: {', '.join(eff_bodies)}")


if __name__ == "__main__":
    main()
