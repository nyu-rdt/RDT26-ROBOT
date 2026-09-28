"""
fix_mtl.py

Cleans up the .mtl written by Blender's OBJ exporter so RViz and Gazebo render it
predictably:

  * every CAD appearance is mapped onto the team's four-colour palette
    (purple printed parts, white wheel rims, silver frame, dark hardware)
  * material names become plain identifiers (no spaces, parentheses or commas)
  * everything is opaque (transparent plastics render as dark slivers in RViz)
  * per-vertex colours are stripped (RViz would use them instead of the material)
  * loose edges and points ("l"/"p" records) are dropped: RViz renders them as red lines
  * only Ka/Kd/Ks/d/illum are kept

Usage: python3 fix_mtl.py <mesh.obj>   (rewrites mesh.obj and mesh.mtl in place)
"""
import os
import re
import sys

OPAQUE_RX = re.compile(r"Opaque[(_](\d+)[,_](\d+)[,_](\d+)")

# Team palette (sRGB). Keep in sync with the <material> blocks in urdf/robot.xacro.
PALETTE = {
    "rdt_purple": (0.60, 0.42, 0.86),   # 3D-printed ABS parts
    "rdt_white":  (0.88, 0.88, 0.88),   # wheel rims
    "rdt_silver": (0.70, 0.70, 0.72),   # aluminium frame, chains, electronics box
    "rdt_dark":   (0.16, 0.16, 0.17),   # mast, motors, camera front, hardware
}
DARK_LUMA = 0.45


def classify(name, kd):
    """Pick a palette entry for a CAD appearance name + Blender base colour."""
    if name in PALETTE:            # already mapped (re-running the script is safe)
        return name
    low = name.lower()
    if "purple" in low:
        return "rdt_purple"
    if "white" in low:             # "ABS (White)" / "ABS_(White)": the wheel rims
        return "rdt_white"
    m = OPAQUE_RX.search(name)
    if m:
        rgb = tuple(int(v) / 255.0 for v in m.groups())
    else:
        rgb = tuple(lin_to_srgb(c) for c in kd)
    luma = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
    return "rdt_dark" if luma < DARK_LUMA else "rdt_silver"


def lin_to_srgb(c):
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


def safe_name(n):
    return re.sub(r"[^A-Za-z0-9]+", "_", n).strip("_") or "mat"


def fix(obj_path):
    mtl_path = os.path.splitext(obj_path)[0] + ".mtl"
    mats = {}
    order = []
    cur = None
    for line in open(mtl_path):
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "newmtl":
            cur = line[7:].strip()
            order.append(cur)
            mats[cur] = {}
        elif cur is not None and parts[0] == "Kd":
            mats[cur]["Kd"] = tuple(float(x) for x in parts[1:4])

    renamed = {name: classify(name, mats[name].get("Kd", (0.7, 0.7, 0.7))) for name in order}
    out = ["# written by fix_mtl.py: four-colour team palette\n"]
    for new, kd in PALETTE.items():
        out.append(f"newmtl {new}\n")
        out.append("Ka {:.4f} {:.4f} {:.4f}\n".format(*kd))
        out.append("Kd {:.4f} {:.4f} {:.4f}\n".format(*kd))
        out.append("Ks 0.1000 0.1000 0.1000\n")
        out.append("d 1.0\n")
        out.append("illum 2\n\n")
    open(mtl_path, "w").writelines(out)

    tmp = obj_path + ".tmp"
    with open(obj_path) as src, open(tmp, "w") as dst:
        for line in src:
            if line.startswith(("l ", "p ")):
                continue   # loose edges/points from the CAD: RViz draws them in its red fallback material
            if line.startswith("v ") and len(line.split()) > 4:
                line = " ".join(line.split()[:4]) + "\n"   # drop per-vertex colours; RViz prefers them over the material
            elif line.startswith("usemtl"):
                old = line[7:].strip()
                line = f"usemtl {renamed.get(old, safe_name(old))}\n"
            elif line.startswith("mtllib"):
                line = f"mtllib {os.path.basename(mtl_path)}\n"
            dst.write(line)
    os.replace(tmp, obj_path)
    return renamed


if __name__ == "__main__":
    for p in sys.argv[1:]:
        r = fix(p)
        print(f"{p}: {len(r)} materials cleaned")
