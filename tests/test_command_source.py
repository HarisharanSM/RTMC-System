#!/usr/bin/env python3
"""Exercise command arbitration against an owned, actual pcan_demo process."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
URL = "http://127.0.0.1:8082"
HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def request(path, method="GET"):
    try:
        with HTTP.open(urllib.request.Request(URL + path, method=method), timeout=.5) as reply:
            return reply.status, json.load(reply)
    except urllib.error.HTTPError as error:
        return error.code, json.load(error)


def command(source, kind="", button="R-up"):
    fields = {"btn": button, "cmd": kind}
    if source is not None:
        fields["source"] = source
    return request("/command?" + urllib.parse.urlencode(fields), "POST")


@contextmanager
def simulator(binary, mode):
    # Never send a request to an unrelated simulator, even if it returns valid JSON.
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", 8082))
    args = [str(binary.resolve()), "--assets", str(ROOT)]
    if mode is not None:
        args += ["--command-source", mode]
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen(args, stdout=log, stderr=log)
        try:
            for _ in range(100):
                if process.poll() is not None:
                    raise AssertionError("Simulator exited during startup")
                try:
                    status, state = request("/state")
                    if status == 200:
                        assert process.poll() is None
                        break
                except (urllib.error.URLError, TimeoutError):
                    time.sleep(.025)
            else:
                raise AssertionError("Simulator did not become ready")
            yield state
        except BaseException:
            log.seek(0)
            print(log.read().decode(errors="replace")[-5000:])
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            assert process.returncode == 0, "Simulator did not shut down cleanly"


def run(binary):
    for arguments in (["--command-source", "invalid"], ["--command-source"], ["--unknown"]):
        result = subprocess.run([str(binary.resolve())] + arguments, capture_output=True, timeout=3)
        assert result.returncode != 0, "Invalid CLI must fail before startup"
    for mode in (None, "ros2"):
        selected = mode or "browser"
        allowed = None if mode is None else "ros2"
        with simulator(binary, mode) as initial:
            assert initial["command_source"] == selected
            assert initial["state"] == "Disarmed"
            rejected = ["ros2", "unknown", ""] if mode is None else [None, "browser", "unknown", ""]
            for other in rejected:
                for kind in ("start", "", "stop"):
                    status, reply = command(other, kind)
                    assert status == 409 and reply["accepted"] is False
            assert request("/state")[1]["sequence"] == initial["sequence"]
            status, start = command(allowed, "start")
            assert status == 200 and start["accepted"]
            assert start["input_sequence"] == 1, "Rejected sources advanced input sequencing"
            assert start["session"] == 1, "Rejected sources allocated sessions"
            # Rejected Stop cannot release the selected owner's session.
            status, reply = command("browser" if mode else "ros2", "stop")
            assert status == 409 and not reply["accepted"]
            for index in range(8):
                time.sleep(.05)
                status, held = command(allowed)
                assert status == 200 and held["session"] == start["session"]
                assert held["input_sequence"] == start["input_sequence"] + index + 1
            moved = request("/state")[1]
            assert moved["sequence"] > initial["sequence"], "Selected source did not drive real motion: " + repr(moved)
            assert moved["feedback_valid"] and moved["pose_source"] == "drive_can_feedback"
            assert command(allowed, "stop")[0] == 200
            stopped = request("/state")[1]
            assert stopped["state"] == "Disarmed" and stopped["speed_dps"] == 0
            assert command(allowed, "start")[1]["session"] != start["session"]
            assert command(allowed, "stop")[0] == 200
            print("PASS command arbitration, CAN sequencing and actual motion:", selected, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/pcan_demo")
    run(parser.parse_args().binary)
