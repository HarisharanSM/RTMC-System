#!/usr/bin/env python3
"""Artifact consistency checks, not machine collision-safety validation."""

import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("reference", ROOT / "tools/generate_collision_reference.py")
ref = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ref)


class ReferenceDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = json.loads(ref.PARAMETERS.read_text())
        cls.scene = ref.build_scene(cls.p)

    def test_simulation_only_and_missing_physical_bounds(self):
        self.assertEqual(self.p["status"], "simulation_only")
        self.assertFalse(self.scene["hardware_authorization"])
        self.assertIsNone(self.p["braking_profile"]["guaranteed_joint_deceleration_rad_s2"])
        self.assertIsNone(self.p["provenance"]["assumptions"]["measurement_uncertainty_m"])

    def test_units_unique_ids_and_frame_references(self):
        self.assertEqual(self.scene["units"], {"length": "m", "angle": "rad", "time": "s"})
        ids = [b["id"] for b in self.scene["bodies"]]
        self.assertEqual(len(ids), len(set(ids)))
        for b in self.scene["bodies"]:
            self.assertIn(b["frame"], self.scene["frames"])
            self.assertTrue(all(math.isfinite(x) and x > 0 for x in b["size_m"]))
            self.assertTrue(all(math.isfinite(x) for x in b["center_m"]))
            self.assertTrue(b["collision_enabled"])
        self.assertEqual(self.scene["pair_policy"]["adjacent_body_exclusions"], [])

    def test_planar_home_and_extension(self):
        for q, expected in [([-180, 180, 0, 0], [0, 0, 1.2]), ([0, 0, 0, 0], [1.5, 0, 1.2])]:
            actual = ref.transform(ref.frames(self.p, q)["carm"], [0, 0, 0])
            for x, y in zip(actual, expected):
                self.assertAlmostEqual(x, y, places=12)

    def test_relative_elbow_and_link_end_agreement(self):
        for a1, a2 in itertools.product([-180, -90, -30, 0, 10], [0, 45, 90, 180]):
            f = ref.frames(self.p, [a1, a2, 0, 0])
            end = ref.transform(f["link2"], [1, 0, 0])
            eof = ref.transform(f["carm"], [0, 0, 0])
            self.assertAlmostEqual(end[0], eof[0], places=12)
            self.assertAlmostEqual(end[1], eof[1], places=12)
        q = [-90, 90, 0, 0]
        eof = ref.transform(ref.frames(self.p, q)["carm"], [0, 0, 0])
        self.assertAlmostEqual(eof[0], 0.75)
        self.assertAlmostEqual(eof[1], -0.75)

    def test_pure_rotation_moves_surface_not_eof(self):
        a = ref.frames(self.p, [0, 0, 0, 0])["carm"]
        b = ref.frames(self.p, [0, 0, 90, 0])["carm"]
        self.assertEqual(ref.transform(a, [0, 0, 0]), ref.transform(b, [0, 0, 0]))
        pa, pb = ref.transform(a, [0, 0, .8]), ref.transform(b, [0, 0, .8])
        self.assertAlmostEqual(math.sqrt(sum((x-y)**2 for x, y in zip(pa, pb))), math.sqrt(1.28))

    def test_transform_rigidity(self):
        for q in self.p["preview_poses_deg"].values():
            for m in ref.frames(self.p, q).values():
                for i in range(3):
                    for j in range(3):
                        self.assertAlmostEqual(sum(m[k][i]*m[k][j] for k in range(3)), float(i == j), places=12)
                self.assertEqual(m[3], [0, 0, 0, 1])

    def test_reference_gap_and_sid(self):
        r = self.p["robot"]
        gap = r["detector_center_m"][2] - r["detector_size_m"][2]/2 - (r["source_center_m"][2] + r["source_size_m"][2]/2)
        self.assertAlmostEqual(gap, .955, places=12)
        self.assertAlmostEqual(r["detector_image_point_m"][2]-r["source_focal_point_m"][2], 1.3)

    def test_annular_sector_enclosure(self):
        r = self.p["robot"]
        arc = [b for b in self.scene["bodies"] if b["id"].startswith("carm_sector_")]
        self.assertEqual(len(arc), r["carm_segments"])
        h = math.radians(r["carm_arc_end_deg"]-r["carm_arc_start_deg"]) / (2*len(arc))
        for b in arc:
            theta = b["rotation_x_rad"]
            for fraction, radius, depth in itertools.product([i/50 for i in range(51)],
                    [r["carm_inner_radius_m"], r["carm_outer_radius_m"]], [-1, 1]):
                angle = theta-h+2*h*fraction
                p = [depth*r["carm_depth_x_m"]/2, radius*math.cos(angle), radius*math.sin(angle)]
                centered = [v-c for v,c in zip(p, b["center_m"])]
                local = ref.transform(ref.rotation("x", -theta), centered)
                self.assertTrue(all(abs(v) <= size/2+1e-12 for v,size in zip(local,b["size_m"])))

    def test_generated_outputs_and_manifest(self):
        for name, expected in ref.outputs(self.p).items():
            self.assertEqual((ref.OUTPUT / name).read_text(), expected, name)
        manifest = json.loads((ref.OUTPUT / "manifest.json").read_text())
        for name, digest in manifest["files"].items():
            self.assertEqual(hashlib.sha256((ref.OUTPUT / name).read_bytes()).hexdigest(), digest)

    def test_obj_boxes_are_closed_outward_and_match_scene(self):
        for path in ref.OUTPUT.rglob("*.obj"):
            objects, points, faces = [], [], []
            offset = 0
            for line in path.read_text().splitlines():
                if line.startswith("o "):
                    if points:
                        objects.append((points, faces))
                        offset += len(points)
                    points, faces = [], []
                elif line.startswith("v "):
                    points.append(list(map(float, line.split()[1:])))
                elif line.startswith("f "):
                    faces.append([int(x)-1-offset for x in line.split()[1:]])
            if points:
                objects.append((points, faces))
            for points, faces in objects:
                self.assertEqual(len(points), 8)
                self.assertEqual(len(faces), 12)
                edges = {}
                volume = 0
                for tri in faces:
                    self.assertTrue(all(0 <= i < 8 for i in tri))
                    a,b,c = (points[i] for i in tri)
                    cross = [b[1]*c[2]-b[2]*c[1], b[2]*c[0]-b[0]*c[2], b[0]*c[1]-b[1]*c[0]]
                    volume += sum(x*y for x,y in zip(a,cross))/6
                    for i,j in zip(tri,tri[1:]+tri[:1]):
                        edge = tuple(sorted((i,j)))
                        edges[edge] = edges.get(edge,0)+1
                self.assertTrue(all(count==2 for count in edges.values()))
                self.assertGreater(volume, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
