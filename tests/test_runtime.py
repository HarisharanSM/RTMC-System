#!/usr/bin/env python3
"""Run the actual simulator HTTP/CAN/drive/avoidance stack. No physical hardware."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[1]
URL = "http://localhost:8082"

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
                      b"drive_can_feedback" in dashboard,
                      "index contains the CAN-driven C-arm canvas without an iframe")
                check(b"10 mm residual gap" in dashboard and b"CRAN/CAUD" in dashboard,
                      "index displays the revised clearance and angular-limit policy")
            with urllib.request.urlopen(URL+"/model/parameters.json") as reply:
                model=json.load(reply)
                check(model["kinematics"]["home_deg"]==[-180,180,0,0,0],
                      "integrated renderer receives the five-axis head-aligned model")
                check(model["kinematics"]["joint_limits_deg"][4]==[-90,90] and
                      model["simulation_pair_margin_m"]==.01,
                      "runtime serves the A5 and 10 mm collision policy")
            with urllib.request.urlopen(URL+"/model/scene.json") as reply:
                scene_bytes=reply.read()
                check(len(json.loads(scene_bytes)["bodies"])>5,
                      "integrated renderer receives generated scene geometry")
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
            # With the revised head-side construction, the support column is
            # the first body to approach the table's head edge. Its centre is
            # I.x-.75 and its head/foot half-width is .08; table left is -.20.
            gap=-.20-(x_m(current)-.75+.08)
            check(current["clear_permits"]>20 and current["sequence"]>20,
                  "worker repeatedly renews permits while actual simulator moves")
            check(current["state"]=="AvoidanceLatched" and current["speed_dps"]==0 and
                  "deadline" not in current["reason"],
                  "collision predictor independently stops the running drive")
            print("Observed support/table predictive-stop gap: %.4f m" % gap, flush=True)
            # The configured residual surface gap is 10 mm. Allow 0.5 mm for
            # fixed-point CAN quantization in this end-to-end observation.
            check(.0095 < gap < .20, "head-side support stops with at least the configured 10 mm clearance")
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
