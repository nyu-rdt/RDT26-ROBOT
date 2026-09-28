"""
glb_to_meshes.py

Converts the rover CAD export (glTF/GLB from Fusion 360) into lightweight
STL meshes for the URDF in nyurdt_description.

Usage (headless Blender):
    blender -b --python glb_to_meshes.py -- <input.glb> <output_dir> [body_target_faces] [wheel_target_faces]

Outputs:
    <output_dir>/body.obj + body.mtl    chassis + excavator + motors + camera mount, ROS base_link frame
    <output_dir>/wheel.obj + wheel.mtl  one wheel, origin at hub center, axle along +Y
    <output_dir>/frames.txt             hub / camera / chain-loop numbers to paste into robot.xacro

Colours come from the Fusion appearances in the GLB (e.g. "ABS (Purple)" for the
3D-printed parts), written to the .mtl files. Both chains (drive and bucket ladder)
are removed and replaced by simple boxes in the xacro; their loop geometry is measured
here and written to frames.txt.

ROS base_link frame convention produced here:
    origin  = ground level, centerd between the four wheel hubs
    +X      = forward (the RealSense / excavator end)
    +Y      = left
    +Z      = up
"""
import bpy
import os
import re
import sys
import math
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fix_mtl  # noqa: E402  (same directory)

# ---------------------------------------------------------------- arguments
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if len(argv) < 2:
    raise SystemExit("usage: blender -b --python glb_to_meshes.py -- in.glb out_dir [body_faces] [wheel_faces]")
IN_GLB, OUT_DIR = argv[0], argv[1]
BODY_TARGET = int(argv[2]) if len(argv) > 2 else 150_000
WHEEL_TARGET = int(argv[3]) if len(argv) > 3 else 15_000

# Parts removed outright. Matched against the part's own name and every
# ancestor assembly name, case-insensitive.
DELETE_RX = re.compile(
    r"screw|bolt|\bnuts?\b|locknut|washer|rivet|insert|bearing|latch"
    r"|roller chain|chain assembly|\b35 chain \(",
    re.I,
)
# Names that would match DELETE_RX but must be kept (the wheel hub says "Bolt Circle").
KEEP_RX = re.compile(r"hex hub|grouser", re.I)

WHEEL_RX = re.compile(r"^Wheel Assembly <(\d)>")
HUB_RX = re.compile(r"^Wheel Hub v7")


def log(*a):
    print("[glb_to_meshes]", *a, flush=True)


def ancestors(o):
    while o is not None:
        yield o
        o = o.parent


def ancestor_names(o):
    return " | ".join(x.name for x in ancestors(o))


def world_bbox_center(objs):
    """Axis-aligned bounds from actual world-space vertices (bound_box inflates rotated parts)."""
    lo = Vector((1e9, 1e9, 1e9))
    hi = Vector((-1e9, -1e9, -1e9))
    for o in objs:
        mw = o.matrix_world
        for v in o.data.vertices:
            w = mw @ v.co
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    return (lo + hi) / 2, lo, hi


def descendants_meshes(root):
    out = []
    st = [root]
    while st:
        x = st.pop()
        if x.type == "MESH":
            out.append(x)
        st.extend(x.children)
    return out


def join(objs, name):
    """Join objs into one mesh object called name (world transforms baked)."""
    objs = [o for o in objs if o.type == "MESH"]
    for o in objs:
        o.data = o.data.copy()  # break instancing so join can bake transforms
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    with bpy.context.temp_override(active_object=objs[0], selected_editable_objects=objs, selected_objects=objs):
        bpy.ops.object.join()
    j = bpy.context.view_layer.objects.active
    j.name = name
    j.parent = None
    # bake world transform into vertices
    j.data.transform(j.matrix_world)
    j.matrix_world = Matrix.Identity(4)
    return j


def face_count(o):
    dg = bpy.context.evaluated_depsgraph_get()
    return len(o.evaluated_get(dg).data.polygons)


def decimate_to(o, target):
    """Planar dissolve, then collapse to roughly `target` faces (modifiers left on; exporter applies)."""
    o.modifiers.clear()
    m = o.modifiers.new("planar", "DECIMATE")
    m.decimate_type = "DISSOLVE"
    m.angle_limit = math.radians(2.0)
    m.delimit = {"NORMAL"}
    n = face_count(o)
    log(f"  {o.name}: {len(o.data.polygons)} faces -> {n} after planar dissolve")
    t = o.modifiers.new("tri", "TRIANGULATE")
    t.quad_method = "BEAUTY"
    t.ngon_method = "BEAUTY"
    n = face_count(o)
    log(f"  {o.name}: {n} triangles after triangulate")
    if n > target:
        c = o.modifiers.new("collapse", "DECIMATE")
        c.decimate_type = "COLLAPSE"
        c.ratio = target / n
        got = face_count(o)
        c.ratio = max(0.001, min(1.0, c.ratio * target / max(got, 1)))
        log(f"  {o.name}: collapse ratio {c.ratio:.3f} -> {face_count(o)} triangles")


def export_obj(o, path):
    """OBJ + MTL, coordinates written as-is (forward=Y, up=Z keeps Blender's Z-up frame)."""
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, apply_modifiers=True,
                          export_materials=True, export_uv=False, export_normals=True,
                          export_triangulated_mesh=True, export_object_groups=False,
                          export_material_groups=False, path_mode="AUTO",
                          forward_axis="Y", up_axis="Z", global_scale=1.0)
    fix_mtl.fix(path)
    log(f"  wrote {path} ({len(o.data.materials)} materials, mtl cleaned)")


# ---------------------------------------------------------------- import
bpy.ops.wm.read_factory_settings(use_empty=True)
log("importing", IN_GLB)
bpy.ops.import_scene.gltf(filepath=IN_GLB)
all_objs = list(bpy.data.objects)
meshes = [o for o in all_objs if o.type == "MESH"]
log(f"imported {len(all_objs)} objects, {len(meshes)} meshes, "
    f"{sum(len(o.data.polygons) for o in meshes)} faces (instanced)")

# ---------------------------------------------------------------- chain loops (measured before removal)
CHAIN_LINK_RX = re.compile(r"roller chain \(|\b35 chain \(", re.I)


def chain_loops(meshes):
    """Cluster chain links by lateral position; fit each loop's center, direction, length, height."""
    links = []
    for o in meshes:
        if CHAIN_LINK_RX.search(ancestor_names(o)):
            mw = o.matrix_world
            c = Vector((0, 0, 0))
            for v in o.data.vertices:
                c += mw @ v.co
            links.append(c / len(o.data.vertices))
    loops = []
    for c in links:
        for lp in loops:
            if abs(lp[0].y - c.y) < 0.02:
                lp.append(c)
                break
        else:
            loops.append([c])
    out = []
    for pts in loops:
        if len(pts) < 20:
            continue
        cen = sum(pts, Vector()) / len(pts)
        # principal direction in the XZ plane
        sxx = sum((p.x - cen.x) ** 2 for p in pts)
        szz = sum((p.z - cen.z) ** 2 for p in pts)
        sxz = sum((p.x - cen.x) * (p.z - cen.z) for p in pts)
        ang = 0.5 * math.atan2(2 * sxz, sxx - szz)
        d = Vector((math.cos(ang), 0, math.sin(ang)))
        n = Vector((-math.sin(ang), 0, math.cos(ang)))
        along = [(p - cen).dot(d) for p in pts]
        across = [(p - cen).dot(n) for p in pts]
        out.append(dict(n=len(pts), center=cen, dir=d, length=max(along) - min(along),
                        height=max(across) - min(across)))
    return out


raw_loops = chain_loops(meshes)

# ---------------------------------------------------------------- hardware / chain removal
removed = 0
removed_faces = 0
for o in meshes:
    names = ancestor_names(o)
    if DELETE_RX.search(names) and not KEEP_RX.search(names):
        removed += 1
        removed_faces += len(o.data.polygons)
        bpy.data.objects.remove(o, do_unlink=True)
meshes = [o for o in bpy.data.objects if o.type == "MESH"]
log(f"removed {removed} hardware/chain parts ({removed_faces} faces); {len(meshes)} meshes remain")

# ---------------------------------------------------------------- wheels
wheel_roots = {}
for o in bpy.data.objects:
    m = WHEEL_RX.match(o.name)
    if m and not any(WHEEL_RX.match(a.name) for a in list(ancestors(o))[1:]):
        wheel_roots[int(m.group(1))] = o
log("wheel roots:", {k: v.name for k, v in wheel_roots.items()})
assert len(wheel_roots) == 4, "expected 4 wheel assemblies"

hubs = {}
wheel_meshes = {}
for i, root in sorted(wheel_roots.items()):
    ms = descendants_meshes(root)
    hub_ms = [o for o in ms if any(HUB_RX.match(a.name) for a in ancestors(o))]
    c, lo, hi = world_bbox_center(hub_ms)
    _, wlo, whi = world_bbox_center(ms)
    hubs[i] = (c, wlo, whi)
    wheel_meshes[i] = ms
    log(f"wheel {i}: hub center {tuple(round(v, 4) for v in c)}  "
        f"bbox {tuple(round(v, 3) for v in wlo)} .. {tuple(round(v, 3) for v in whi)}")

# Blender frame after glTF import: Z up, X = glTF X, Y = -glTF Z.
# Forward for the robot is -X here (RealSense end), so ROS = rotate 180 deg about Z.
ground_z = min(h[1].z for h in hubs.values())
x0 = sum(h[0].x for h in hubs.values()) / 4
y0 = sum(h[0].y for h in hubs.values()) / 4
to_ros = Matrix.Rotation(math.pi, 4, "Z") @ Matrix.Translation((-x0, -y0, -ground_z))
log(f"ground z={ground_z:.4f}  center x0={x0:.4f} y0={y0:.4f}")

# one shared wheel mesh from wheel 1, origin at its hub center
w = join(wheel_meshes[1], "wheel")
hub_c = hubs[1][0]
w.data.transform(Matrix.Rotation(math.pi, 4, "Z") @ Matrix.Translation(-hub_c))
decimate_to(w, WHEEL_TARGET)
export_obj(w, f"{OUT_DIR}/wheel.obj")
wheel_radius = max((whi.z - wlo.z) for _, wlo, whi in hubs.values()) / 2
wheel_width = max((whi.y - wlo.y) for _, wlo, whi in hubs.values())
for i in (2, 3, 4):
    for o in wheel_meshes[i]:
        bpy.data.objects.remove(o, do_unlink=True)
bpy.data.objects.remove(w, do_unlink=True)

# ---------------------------------------------------------------- camera (before join)
cam_c = None
cam = [o for o in bpy.data.objects if o.name.startswith("Intel Realsense")]
log("realsense objects:", [o.name for o in cam])
if cam:
    cm = descendants_meshes(cam[0])
    if cm:
        c, clo, chi = world_bbox_center(cm)
        cam_c = to_ros @ c
        log(f"realsense body center (ROS) {tuple(round(v, 4) for v in cam_c)} size {tuple(round(v, 3) for v in (chi - clo))}")

# ---------------------------------------------------------------- body
body_meshes = [o for o in bpy.data.objects if o.type == "MESH"]
log(f"joining {len(body_meshes)} body meshes")
b = join(body_meshes, "body")
b.data.transform(to_ros)
log(f"body: {len(b.data.polygons)} faces before decimation")
decimate_to(b, BODY_TARGET)
export_obj(b, f"{OUT_DIR}/body.obj")

# ---------------------------------------------------------------- report
with open(f"{OUT_DIR}/frames.txt", "w") as f:
    f.write("# ROS base_link frame: ground level, centerd on wheel hubs, +X forward, +Y left\n")
    for i, (c, _, _) in sorted(hubs.items()):
        r = to_ros @ c
        f.write(f"wheel{i}_hub  x={r.x:.4f} y={r.y:.4f} z={r.z:.4f}\n")
    f.write(f"wheel_radius {wheel_radius:.4f}\nwheel_width {wheel_width:.4f}\n")
    if cam_c is not None:
        f.write(f"realsense_body_center x={cam_c.x:.4f} y={cam_c.y:.4f} z={cam_c.z:.4f}\n")
    f.write("# chain loops (ROS frame): center, pitch = angle of the long axis about +Y, loop length and height\n")
    R = to_ros.to_3x3()
    for i, lp in enumerate(sorted(raw_loops, key=lambda l: (-l["length"], l["center"].y))):
        c = to_ros @ lp["center"]
        d = R @ lp["dir"]
        if d.x < 0:
            d = -d
        pitch = -math.atan2(d.z, d.x)   # URDF pitch about +Y rotates +X toward -Z
        f.write(f"chain{i} links={lp['n']} x={c.x:.4f} y={c.y:.4f} z={c.z:.4f} "
                f"pitch={pitch:.4f} length={lp['length']:.4f} height={lp['height']:.4f}\n")
log(open(f"{OUT_DIR}/frames.txt").read())
log("done")
