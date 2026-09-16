#!/usr/bin/env python3
"""Run the actual simulator HTTP/CAN/drive/avoidance stack. No physical hardware."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import time
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[1]
URL = "http://localhost:8082"
REFERENCE_SPEC = importlib.util.spec_from_file_location(
    "collision_reference", ROOT / "tools/generate_collision_reference.py")
REFERENCE = importlib.util.module_from_spec(REFERENCE_SPEC)
REFERENCE_SPEC.loader.exec_module(REFERENCE)

def state():
    with urllib.request.urlopen(URL + "/state", timeout=2) as reply:
        return json.load(reply)

def command(button, kind=""):
    request = urllib.request.Request(URL + "/command?btn=" + button + "&cmd=" + kind,
                                     method="POST")
    with urllib.request.urlopen(request, timeout=2) as reply:
        response=json.load(reply)
        assert response["accepted"]
        return response

def x_m(s):
    q = list(map(math.radians, s["axles_deg"]))
    return -.25 + .75*math.cos(q[0]) + math.cos(q[0]+q[1])

def dot(a, b):
    return sum(x*y for x,y in zip(a,b))

def sub(a, b):
    return [x-y for x,y in zip(a,b)]

def norm(a):
    return math.sqrt(dot(a,a))

def box_at(body, frame):
    local=REFERENCE.multiply(REFERENCE.translation(*body["center_m"]),
        REFERENCE.multiply(REFERENCE.multiply(
            REFERENCE.rotation("x",body["rotation_x_rad"]),
            REFERENCE.rotation("y",body["rotation_y_rad"])),
            REFERENCE.rotation("z",body["rotation_z_rad"])))
    pose=REFERENCE.multiply(frame,local)
    return {"center":[pose[i][3] for i in range(3)],
            "axes":[[pose[i][j] for i in range(3)] for j in range(3)],
            "half":[v/2 for v in body["size_m"]]}

def vertices(box):
    return [[box["center"][i]+sum(box["axes"][j][i]*box["half"][j]*sign[j]
            for j in range(3)) for i in range(3)]
            for sign in ((sx,sy,sz) for sx in (-1,1) for sy in (-1,1) for sz in (-1,1))]

def point_box_distance2(point, box):
    delta=sub(point,box["center"]); result=0
    for axis,half in zip(box["axes"],box["half"]):
        result += max(0,abs(dot(delta,axis))-half)**2
    return result

def segment_distance2(p0,p1,q0,q1):
    d1=sub(p1,p0);d2=sub(q1,q0);r=sub(p0,q0);a=dot(d1,d1);e=dot(d2,d2);f=dot(d2,r);eps=1e-15;s=t=0
    if a<=eps and e<=eps:return dot(r,r)
    if a<=eps:t=max(0,min(1,f/e))
    else:
        c=dot(d1,r)
        if e<=eps:s=max(0,min(1,-c/a))
        else:
            b=dot(d1,d2);den=a*e-b*b
            if den>eps:s=max(0,min(1,(b*f-c*e)/den))
            t=(b*s+f)/e
            if t<0:t=0;s=max(0,min(1,-c/a))
            elif t>1:t=1;s=max(0,min(1,(b-c)/a))
    closest=[r[i]+d1[i]*s-d2[i]*t for i in range(3)]
    return dot(closest,closest)

def cross(a,b):
    return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]

def boxes_overlap(a,b):
    axes=a["axes"]+b["axes"]+[cross(x,y) for x in a["axes"] for y in b["axes"]]
    delta=sub(b["center"],a["center"])
    for axis in axes:
        length=norm(axis)
        if length<=1e-10:continue
        axis=[x/length for x in axis]
        radius=lambda box:sum(h*abs(dot(v,axis)) for h,v in zip(box["half"],box["axes"]))
        if abs(dot(delta,axis))>radius(a)+radius(b):return False
    return True

def box_distance(a,b):
    if boxes_overlap(a,b):return 0
    av,bv=vertices(a),vertices(b);best=float("inf")
    for point in av:best=min(best,point_box_distance2(point,b))
    for point in bv:best=min(best,point_box_distance2(point,a))
    for i in range(8):
        for bit in (1,2,4):
            j=i^bit
            if i>j:continue
            for k in range(8):
                for other_bit in (1,2,4):
                    l=k^other_bit
                    if k<=l:best=min(best,segment_distance2(av[i],av[j],bv[k],bv[l]))
    return math.sqrt(max(0,best))

def minimum_checked_gap(model, scene, axles):
    transforms=REFERENCE.frames(model,axles)
    boxes=[box_at(body,transforms[body["frame"]]) for body in scene["bodies"]]
    excluded={frozenset(pair) for pair in scene["pair_policy"]["adjacent_body_exclusions"]}
    best=(float("inf"),"","")
    for i,lhs in enumerate(scene["bodies"]):
        for j in range(i+1,len(scene["bodies"])):
            rhs=scene["bodies"][j]
            if lhs["rigid_body"]==rhs["rigid_body"]:continue
            if lhs["frame"]==rhs["frame"]=="world":continue
            if frozenset((lhs["rigid_body"],rhs["rigid_body"])) in excluded:continue
            gap=box_distance(boxes[i],boxes[j])
            if gap<best[0]:best=(gap,lhs["id"],rhs["id"])
    return best

def named_pair_gap(model, scene, axles, first, second):
    transforms=REFERENCE.frames(model,axles)
    bodies={body["id"]:body for body in scene["bodies"]}
    lhs,rhs=bodies[first],bodies[second]
    return box_distance(box_at(lhs,transforms[lhs["frame"]]),
                        box_at(rhs,transforms[rhs["frame"]]))

def check(value, description):
    if not value:
        raise AssertionError(description)
    print("PASS:", description, flush=True)

def wait_for(predicate, timeout=1):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        current = state()
        if predicate(current):
            return current
        time.sleep(.01)
    raise AssertionError("Timed out waiting for expected runtime state")

def expect_startup_failure(binary, arguments, expected_message, description):
    process = subprocess.run([str(binary.resolve()), *arguments], capture_output=True,
                             timeout=3, check=False)
    output = process.stdout + process.stderr
    check(process.returncode != 0 and expected_message.encode() in output, description)

def run(binary):
    # Never drive an unrelated process already listening on the test port.
    try:
        urllib.request.urlopen(URL + "/state", timeout=.2).close()
    except (urllib.error.URLError, TimeoutError):
        pass
    else:
        raise RuntimeError("Port 8082 already in use. Stop the existing simulator before this test.")

    # Startup faults must fail closed before the normal runtime is launched.
    with tempfile.TemporaryDirectory() as empty_assets:
        expect_startup_failure(binary, ["--assets", empty_assets],
                               "Missing dashboard assets",
                               "missing assets prevent an unsupervised startup")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
        occupied.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        occupied.bind(("127.0.0.1", 8082))
        occupied.listen(1)
        expect_startup_failure(binary, ["--assets", str(ROOT)],
                               "Unable to listen on localhost:8082",
                               "occupied control port produces an explicit startup failure")

    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([str(binary.resolve()), "--assets", str(ROOT)],
                                   stdout=log, stderr=log)
        completed = False
        try:
            for _ in range(100):
                if process.poll() is not None:
                    raise RuntimeError("Simulator exited during startup")
                try:
                    initial=state()
                    break
                except (urllib.error.URLError, TimeoutError):
                    time.sleep(.05)
            else:
                raise RuntimeError("Simulator did not start")
            check(initial["state"]=="Disarmed" and initial["collision_enabled"] and
                  len(initial["axles_deg"])==5 and initial["protocol_version"]==3,
                  "startup has collision supervision and revision-3 five-axis telemetry")
            check(initial["feedback_valid"] and initial["pose_source"]=="drive_can_feedback" and
                  initial["pose_sequence"]==initial["sequence"] and initial["can_feedback_frames"]>=7,
                  "home pose is assembled from a complete drive-originated CAN sample")
            with urllib.request.urlopen(URL) as reply:
                dashboard=reply.read()
                check(b'<canvas id="carm-view"' in dashboard and b'<iframe' not in dashboard and
                      b"drive_can_feedback" in dashboard and b'data-view="side"' in dashboard and
                      b'data-view="back"' in dashboard and b'data-view="top"' in dashboard,
                      "index contains the CAN-driven C-arm canvas without an iframe")
                check(b"10 mm residual gap" in dashboard and b"CRAN/CAUD" in dashboard,
                      "index displays the revised clearance and angular-limit policy")
            with urllib.request.urlopen(URL+"/model/parameters.json") as reply:
                model=json.load(reply)
                check(model["schema_version"]==5 and model["model_id"]=="rtmc-drawing-assembly-v5" and
                      model["kinematics"]["home_deg"]==[-180,180,0,0,0],
                      "integrated renderer receives the revision-5 head-aligned model")
                check(model["kinematics"]["joint_limits_deg"][4]==[-90,90] and
                      model["simulation_pair_margin_m"]==.01,
                      "runtime serves the A5 and 10 mm collision policy")
            with urllib.request.urlopen(URL+"/model/scene.json") as reply:
                scene_bytes=reply.read();scene=json.loads(scene_bytes)
                ids={body["id"] for body in scene["bodies"]}
                check({"support_column","a3_housing","upper_boom","a4_carrier_left",
                       "a5_bearing_left","source_housing","detector_housing"} <= ids,
                      "integrated renderer receives the connected drawing-based assembly")
            with urllib.request.urlopen(URL+"/model/manifest.json") as reply:
                manifest=json.load(reply)
                check(hashlib.sha256(scene_bytes).hexdigest()==manifest["files"]["scene.json"] and
                      model["model_id"]==manifest["model_id"],
                      "integrated renderer assets agree with the generated manifest")
            initial_sequence=initial["sequence"]
            # A lost browser/joystick stream must stop without any more drive calls.
            first_start=command("R-up","start")
            check(first_start["protocol_version"]==3 and first_start["session"]>0 and
                  first_start["input_sequence"]>0,
                  "UI press is encoded with revision, sequence and session")
            last_input_sequence=first_start["input_sequence"]
            for _ in range(10):
                time.sleep(.05); held=command("R-up")
                assert held["session"]==first_start["session"] and held["input_sequence"]>last_input_sequence
                last_input_sequence=held["input_sequence"]
                if state()["sequence"] > initial_sequence:
                    break
            moved=state()
            check(moved["sequence"]>initial_sequence and moved["feedback_valid"] and
                  moved["can_feedback_frames"]>initial["can_feedback_frames"],
                  "clear preflight commits movement and returns it through CAN feedback")
            watchdog=wait_for(lambda s:s["state"]=="AvoidanceLatched")
            check(watchdog["speed_dps"]==0 and "deadline" in watchdog["reason"],
                  "independent watchdog stops on missing renewal")
            command("none","stop")
            check(state()["state"]=="Disarmed", "controller Stop acknowledges watchdog latch")
            second_start=command("R-up","start")
            check(second_start["session"]!=first_start["session"],
                  "Stop retires the input session and fresh press allocates another")
            for _ in range(400):
                time.sleep(.05); command("R-up")
                current=state()
                q=current["axles_deg"]
                assert abs(q[0]+q[1]+q[2])<=3e-4, "A3 alignment exceeds CAN quantization bound"
                if current["state"]=="AvoidanceLatched":
                    break
            else:
                raise AssertionError("No avoidance stop on approach to fixed table pedestal")
            pair_match=re.search(r"\(([^()]+) / ([^()]+)\)$",current["reason"])
            check(pair_match is not None,"runtime stop reports the limiting 3D body pair")
            first_body,second_body=pair_match.groups()
            gap=named_pair_gap(model,scene,current["axles_deg"],first_body,second_body)
            check(current["clear_permits"]>20 and current["sequence"]>20,
                  "worker repeatedly renews permits while actual simulator moves")
            check(current["state"]=="AvoidanceLatched" and current["speed_dps"]==0 and
                  "deadline" not in current["reason"],
                  "collision predictor independently stops the running drive")
            print("Observed limiting-pair gap: %.4f m (%s / %s)" %
                  (gap,first_body,second_body), flush=True)
            # The configured residual surface gap is 10 mm. Allow 0.5 mm for
            # fixed-point CAN quantization in this end-to-end observation.
            check(.0095 < gap < .20, "drawing-based 3D model stops with at least the configured 10 mm clearance")
            sequence=current["sequence"]
            command("R-up","start")
            for _ in range(3):
                command("R-up"); time.sleep(.05)
            check(state()["sequence"]==sequence, "held input and duplicate Start cannot restart collision latch")
            command("none","stop")
            check(state()["state"]=="Disarmed", "controller Stop acknowledges collision latch")
            reverse_start=command("R-down","start")
            check(reverse_start["session"]!=second_start["session"],
                  "reverse begins in a fresh controller input session")
            for _ in range(10):
                time.sleep(.05); command("R-down")
                if state()["sequence"]>sequence:
                    break
            check(state()["sequence"]>sequence, "fresh reverse preflight allows safe movement away")
            command("none","stop")
            print("Observed table clearance: %.3f m; X: %.3f m" % (gap,x_m(current)),flush=True)
            completed = True
        except Exception:
            log.seek(0)
            print(log.read().decode(errors="replace")[-6000:])
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        if completed:
            check(process.returncode == 0, "SIGTERM performs a clean supervised shutdown")

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",type=Path,default=ROOT/"build/pcan_demo")
    run(parser.parse_args().binary)
