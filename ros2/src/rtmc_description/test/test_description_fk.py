import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
PKG = Path(__file__).resolve().parents[1]


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def rot_axis(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
            [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
            [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]


def rpy_matrix(rpy):
    r, p, y = rpy
    rx = rot_axis((1, 0, 0), r)
    ry = rot_axis((0, 1, 0), p)
    rz = rot_axis((0, 0, 1), y)
    return mm(mm(rz, ry), rx)


def urdf_frames(angles):
    tree = ET.parse(PKG / 'urdf' / 'rtmc.urdf').getroot()
    children = {}
    links = {link.get('name') for link in tree.findall('link')}
    child_links = set()
    assert {joint.get('name') for joint in tree.findall('joint')
            if joint.get('type') != 'fixed'} == set(angles)
    for joint in tree.findall('joint'):
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        assert parent in links and child in links and child not in child_links
        child_links.add(child)
        children.setdefault(parent, []).append((child, joint))
    transforms = {'rtmc_patient': ([[1,0,0],[0,1,0],[0,0,1]], [0,0,0])}
    pending = ['rtmc_patient']
    while pending:
        parent_name = pending.pop()
        pr, pt = transforms[parent_name]
        for child_name, joint in children.get(parent_name, []):
            assert child_name not in transforms, 'URDF tree has a cycle'
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0,0,0]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0,0,0]
            jr, jt = rpy_matrix(rpy), xyz
            kind = joint.get('type')
            if kind in ('revolute', 'continuous'):
                axis = [float(x) for x in joint.find('axis').get('xyz').split()]
                qr = rot_axis(axis, angles[joint.get('name')])
                jr = mm(jr, qr)
            transforms[child_name] = (mm(pr, jr), [a+b for a,b in zip(pt, mv(pr, jt))])
            pending.append(child_name)
    assert set(transforms) == links, 'URDF contains disconnected links'
    return transforms


def test_description_matches_cpp_drive_and_collision_fk():
    compiler = os.environ.get('CXX', 'c++')
    with tempfile.TemporaryDirectory(prefix='rtmc-description-') as tmp:
        exe = Path(tmp) / 'fk_reference'
        subprocess.run([
            compiler, '-std=c++17', '-O2',
            '-I' + str(ROOT / 'includes'), '-I' + str(ROOT / 'drive' / 'include'),
            '-I' + str(ROOT / 'collision' / 'include'),
            str(PKG / 'test' / 'fk_reference.cpp'),
            str(ROOT / 'drive' / 'src' / 'cDriveCalculator.cpp'),
            str(ROOT / 'collision' / 'src' / 'cBodyKinematics.cpp'),
            str(ROOT / 'collision' / 'src' / 'cCollisionMath.cpp'), '-o', str(exe),
        ], check=True)
        cases_deg = [
            (-180, 180, 0, 0, 0),      # closed home
            (-110, 145, -35, 12, -18), # interior with independent A3
            (-65, 100, 25, 0, 0),      # retained nonzero heading
            (-130, 160, -22, 55, 0),   # LAO only
            (-95, 135, 12, 0, -48),    # CRAN only
        ]
        payload = ''.join(' '.join(str(math.radians(v)) for v in q) + '\n' for q in cases_deg)
        output = subprocess.run([str(exe)], input=payload, text=True, capture_output=True, check=True).stdout.splitlines()

    frame_names = ['rtmc_patient', 'link1', 'link2', 'column', 'boom', 'a4_carrier', 'a5_carrier', 'c_arm']
    body_names = ['World', 'Link1', 'Link2', 'Column', 'Boom', 'A4Carrier', 'A5Carrier', 'CArm']
    assert len(output) == len(cases_deg), f"C++ helper returned {len(output)} cases"
    for case, line in zip(cases_deg, output):
        values = [float(x) for x in line.split()]
        fk_x, fk_y, fk_yaw = values[:3]
        assert len(values) == 3 + 8 * 12, f"unexpected C++ FK row width: {len(values)}"
        nums = values[3:]
        refs = {}
        for name in body_names:
            segment, nums = nums[:12], nums[12:]
            refs[name] = (segment[:3], [segment[3:6], segment[6:9], segment[9:12]])
        qrad = {f'A{i+1}': math.radians(case[i]) for i in range(5)}
        urdf = urdf_frames(qrad)
        center = urdf['c_arm'][1]
        assert max(abs(a-b) for a,b in zip(urdf['imaging_center'][1], refs['CArm'][0])) < 1e-9
        heading = math.radians(sum(case[:3]))
        imaging_r = urdf['imaging_center'][0]
        assert max(abs(imaging_r[i][j]-rot_axis((0,0,1), heading)[i][j]) for i in range(3) for j in range(3)) < 1e-9
        assert abs(center[0] - fk_x) < 1e-9
        assert abs(center[1] - fk_y) < 1e-9
        assert abs(math.atan2(urdf['imaging_center'][0][1][0], urdf['imaging_center'][0][0][0]) - fk_yaw) < 1e-9
        for link_name, body_name in zip(frame_names, body_names):
            actual_r, actual_t = urdf[link_name]
            ref_t, ref_r = refs[body_name]
            assert max(abs(a-b) for a,b in zip(actual_t, ref_t)) < 1e-9, (link_name, actual_t, ref_t)
            assert max(abs(actual_r[i][j]-ref_r[i][j]) for i in range(3) for j in range(3)) < 1e-9, link_name


if __name__ == "__main__":
    test_description_matches_cpp_drive_and_collision_fk()
