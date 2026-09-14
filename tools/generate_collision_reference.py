#!/usr/bin/env python3
"""Generate synthetic collision primitives, OBJ assets and scene metadata.

Standard library only. Does not implement runtime collision avoidance. Sources
are parameters.json and this generator; outputs are reproducible and checked in.
"""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARAMETERS = ROOT / "data/collision/reference/parameters.json"
OUTPUT = ROOT / "data/collision/reference/generated"


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4))
             for j in range(4)] for i in range(4)]


def translation(x, y, z):
    result = identity()
    result[0][3], result[1][3], result[2][3] = x, y, z
    return result


def rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    result = identity()
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    result[i][i] = result[j][j] = c
    result[i][j], result[j][i] = -s, s
    return result


def transform(t, p):
    return [sum(t[i][j] * p[j] for j in range(3)) + t[i][3]
            for i in range(3)]


def frames(parameters, degrees):
    k = parameters["kinematics"]
    a1, a2, a3, a4, a5 = map(math.radians, degrees)
    bx, by = k["base_xy_m"]
    elbow = [bx + k["link1_m"] * math.cos(a1),
             by + k["link1_m"] * math.sin(a1)]
    eof = [elbow[0] + k["link2_m"] * math.cos(a1 + a2),
           elbow[1] + k["link2_m"] * math.sin(a1 + a2)]
    heading = rotation("z", a1 + a2)
    aligned = rotation("z", a1 + a2 + a3)
    return {
        "world": identity(),
        "link1": multiply(translation(bx, by, k["link1_center_z_m"]), rotation("z", a1)),
        "link2": multiply(translation(*elbow, k["link2_center_z_m"]), heading),
        "eof_support": multiply(translation(*eof, k["link2_center_z_m"]), aligned),
        "carm": multiply(multiply(multiply(translation(*eof, k["isocenter_z_m"]),
                                             aligned), rotation("y", a4)), rotation("x", a5)),
    }


def build_scene(p):
    k, r, t = p["kinematics"], p["robot"], p["table"]
    bodies = []

    def box(name, frame, group, size, center, rx=0.0):
        bodies.append({
            "id": name, "frame": frame, "rigid_body": group,
            "mobility": "static" if frame == "world" else "articulated",
            "shape": "box", "size_m": size, "center_m": center,
            "rotation_x_rad": rx, "dimension_status": "synthetic_unmeasured",
            "collision_enabled": True,
        })

    bx, by = k["base_xy_m"]
    box("robot_base", "world", "robot_base", r["base_size_m"], [bx, by, r["base_size_m"][2] / 2])
    box("link1_housing", "link1", "link1", [k["link1_m"], *r["link1_cross_section_yz_m"]],
        [k["link1_m"] / 2, 0, 0])
    box("elbow_housing", "link1", "link1", r["elbow_size_m"],
        [k["link1_m"], 0, (k["link2_center_z_m"] - k["link1_center_z_m"]) / 2])
    box("link2_housing", "link2", "link2", [k["link2_m"], *r["link2_cross_section_yz_m"]],
        [k["link2_m"] / 2, 0, 0])
    rear, width, height = r["support_rear_offset_m"], r["support_width_m"], r["support_height_m"]
    box("support_rear_beam", "eof_support", "alignment", [width, rear + width, height], [0, -rear / 2, 0])
    rise = k["isocenter_z_m"] - k["link2_center_z_m"]
    box("support_column", "eof_support", "alignment", [width, width, rise], [0, -rear, rise / 2])

    # Each OBB encloses one full annular sector: in its radial/tangent basis,
    # radial coordinate lies [inner*cos(h), outer] and tangent lies +/-outer*sin(h).
    # This encloses the analytic reference arc, not unknown physical OEM geometry.
    start, end = map(math.radians, (r["carm_arc_start_deg"], r["carm_arc_end_deg"]))
    half = (end - start) / (2 * r["carm_segments"])
    inner, outer = r["carm_inner_radius_m"], r["carm_outer_radius_m"]
    lo, hi = inner * math.cos(half), outer
    radial_center = (lo + hi) / 2
    for i in range(r["carm_segments"]):
        theta = start + (2 * i + 1) * half
        box("carm_sector_%02d" % i, "carm", "carm",
            [r["carm_depth_x_m"], hi - lo, 2 * outer * math.sin(half)],
            [0, radial_center * math.cos(theta), radial_center * math.sin(theta)], theta)
    box("detector_housing", "carm", "carm", r["detector_size_m"], r["detector_center_m"])
    box("source_housing", "carm", "carm", r["source_size_m"], r["source_center_m"])

    tx, ty = t["center_xy_m"]
    top = t["top_center_z_m"] + t["top_size_m"][2] / 2
    base_height = t["base_size_m"][2]
    box("table_top", "world", "fixed_table", t["top_size_m"], [tx, ty, t["top_center_z_m"]])
    box("table_pedestal", "world", "fixed_table", t["pedestal_size_m"],
        [t["pedestal_center_x_m"], ty, base_height + t["pedestal_size_m"][2] / 2])
    box("table_base", "world", "fixed_table", t["base_size_m"], [t["pedestal_center_x_m"], ty, base_height / 2])
    mattress_height = t["mattress_size_m"][2]
    box("mattress", "world", "fixed_table", t["mattress_size_m"], [tx, ty, top + mattress_height / 2])
    if p["patient_fixture"]["enabled"]:
        size = p["patient_fixture"]["size_m"]
        box("patient_test_envelope", "world", "patient_fixture", size,
            p["patient_fixture"]["center_m"])
        box("patient_head", "world", "patient_fixture", p["patient_fixture"]["head_size_m"],
            p["patient_fixture"]["head_center_m"])
    size = p["environment"]["floor_size_m"]
    box("floor", "world", "floor", size, [0.5, 0, -size[2] / 2])
    return {
        "schema_version": 2, "model_id": p["model_id"], "status": p["status"],
        "hardware_authorization": False, "units": p["units"],
        "transform_layout": "row-major 4x4; column vectors; local point to world",
        "frames": ["world", "link1", "link2", "eof_support", "carm"],
        "bodies": bodies,
        "pair_policy": {
            "default": "check all robot-environment and inter-body robot pairs",
            "same_rigid_body": "exclude internal primitive overlaps",
            "adjacent_body_exclusions": [],
            "note": "No blanket adjacent-link exclusions. Intended joints/floor support contacts require reviewed local contact masks before runtime use. Preview poses are not certified collision-free."
        },
        "pair_margin_m": p["simulation_pair_margin_m"],
        "uncertainty_status": "unmeasured; margin is illustrative, not a physical bound",
    }


def body_vertices(body):
    local = multiply(translation(*body["center_m"]), rotation("x", body["rotation_x_rad"]))
    return [transform(local, [sign[i] * body["size_m"][i] / 2 for i in range(3)])
            for sign in itertools.product((-1, 1), repeat=3)]


# Outward-facing triangles; vertex ordering matches itertools.product above.
TRIANGLES = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
             (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
             (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]


def obj_text(bodies, transforms=None):
    lines = ["# RTMC synthetic reference. SIMULATION ONLY; meters, Z up.",
             "# Each object is a closed box. Compound objects may overlap at joints."]
    offset = 1
    for body in bodies:
        lines.append("o " + body["id"])
        points = body_vertices(body)
        if transforms is not None:
            points = [transform(transforms[body["frame"]], v) for v in points]
        lines.extend("v %.10f %.10f %.10f" % tuple(v) for v in points)
        lines.extend("f %d %d %d" % tuple(offset + i for i in tri) for tri in TRIANGLES)
        offset += 8
    return "\n".join(lines) + "\n"


def json_text(value):
    return json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"


def outputs(p):
    scene = build_scene(p)
    files = {"scene.json": json_text(scene)}
    for frame in scene["frames"]:
        files["frames/" + frame + ".obj"] = obj_text([b for b in scene["bodies"] if b["frame"] == frame])
    for name, q in p["preview_poses_deg"].items():
        transforms = frames(p, q)
        files["poses/" + name + ".json"] = json_text({
            "status": "visualization_only_not_clearance_certified", "joint_angles_deg": q,
            "world_from_frame": transforms,
        })
        files["poses/" + name + ".obj"] = obj_text(scene["bodies"], transforms)
    template = ROOT / "tools/collision_reference_viewer.html"
    files["viewer.html"] = template.read_text().replace(
        "__REFERENCE_DATA__", json.dumps({"parameters": p, "scene": scene}, allow_nan=False))
    files["manifest.json"] = json_text({
        "model_id": p["model_id"], "status": "simulation_only", "hardware_authorization": False,
        "parameter_sha256": hashlib.sha256(PARAMETERS.read_bytes()).hexdigest(),
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "viewer_template_sha256": hashlib.sha256(template.read_bytes()).hexdigest(),
        "files": {name: hashlib.sha256(value.encode()).hexdigest() for name, value in sorted(files.items())},
    })
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify generated outputs without writing")
    args = parser.parse_args()
    p = json.loads(PARAMETERS.read_text())
    files = outputs(p)
    if args.check:
        mismatches = [name for name, data in files.items()
                      if not (OUTPUT / name).is_file() or (OUTPUT / name).read_text() != data]
        extra = sorted(str(path.relative_to(OUTPUT)) for path in OUTPUT.rglob("*")
                       if path.is_file() and str(path.relative_to(OUTPUT)) not in files)
        if mismatches or extra:
            raise SystemExit("Generated output mismatch: %s; unexpected files: %s" % (mismatches, extra))
        print("Verified %d reproducible generated files" % len(files))
    else:
        for name, value in files.items():
            destination = OUTPUT / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(value)
        print("Generated %d reference files in %s" % (len(files), OUTPUT))


if __name__ == "__main__":
    main()
