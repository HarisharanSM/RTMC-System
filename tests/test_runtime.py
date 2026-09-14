#!/usr/bin/env python3
"""Run the actual simulator HTTP/CAN/drive/avoidance stack. No physical hardware."""
import argparse
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
        assert json.load(reply)["accepted"]

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
                  len(initial["axles_deg"])==5, "startup has collision supervision and five axle telemetry")
            with urllib.request.urlopen(URL) as reply:
                check(b'/viewer?live=1' in reply.read(), "joystick dashboard embeds live 3D display")
            with urllib.request.urlopen(URL+"/viewer?live=1") as reply:
                check(b'Patient head' in reply.read(), "viewer includes patient-head coordinate marker")
            # A lost browser/joystick stream must stop without any more drive calls.
            command("R-up","start")
            for _ in range(10):
                time.sleep(.05); command("R-up")
                if state()["sequence"] > 0:
                    break
            check(state()["sequence"]>0, "clear preflight permits actual movement")
            watchdog=wait_for(lambda s:s["state"]=="AvoidanceLatched")
            check(watchdog["speed_dps"]==0 and "deadline" in watchdog["reason"],
                  "independent watchdog stops on missing renewal")
            command("none","stop")
            check(state()["state"]=="Disarmed", "controller Stop acknowledges watchdog latch")
            command("R-up","start")
            for _ in range(400):
                time.sleep(.05); command("R-up")
                current=state()
                q=current["axles_deg"]
                assert abs(q[0]+q[1]+q[2])<1e-8, "A3 alignment drift"
                if current["state"]=="AvoidanceLatched":
                    break
            else:
                raise AssertionError("No avoidance stop on approach to fixed table pedestal")
            # Source right face is x+.19; pedestal left face is 1.50 m.
            gap=1.50-(x_m(current)+.19)
            check(current["clear_permits"]>20 and current["sequence"]>20,
                  "worker repeatedly renews permits while actual simulator moves")
            check(current["state"]=="AvoidanceLatched" and current["speed_dps"]==0 and
                  "deadline" not in current["reason"],
                  "collision predictor independently stops the running drive")
            check(.02 < gap < .20, "source stops with positive clearance before pedestal contact")
            sequence=current["sequence"]
            command("R-up","start")
            for _ in range(3):
                command("R-up"); time.sleep(.05)
            check(state()["sequence"]==sequence, "held input and duplicate Start cannot restart collision latch")
            command("none","stop")
            check(state()["state"]=="Disarmed", "controller Stop acknowledges collision latch")
            command("R-down","start")
            for _ in range(10):
                time.sleep(.05); command("R-down")
                if state()["sequence"]>sequence:
                    break
            check(state()["sequence"]>sequence, "fresh reverse preflight allows safe movement away")
            command("none","stop")
            print("Observed pedestal clearance: %.3f m; X: %.3f m" % (gap,x_m(current)),flush=True)
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
