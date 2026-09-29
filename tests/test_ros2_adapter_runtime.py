#!/usr/bin/env python3
"""Validate the ROS-independent bridge policy against the real C++ simulator.

This is not a ROS discovery/serialization test. Those run in the Jazzy smoke
test; this test provides actual CAN/drive evidence without installing ROS.
"""
import argparse
from pathlib import Path
import sys
import time

from test_command_source import ROOT, command, request, simulator

sys.path.insert(0, str(ROOT / "ros2/src/rtmc_ros2"))
from rtmc_ros2.policy import JogPolicy, feedback_check
from rtmc_ros2.transport import SimulatorClient


def run(binary):
    with simulator(binary, "ros2") as initial:
        policy = JogPolicy()
        sequence = 0
        dispatches = []
        transport = SimulatorClient()

        def dispatch(kind, direction):
            dispatches.append(kind)
            return transport.dispatch(kind, direction)

        def send(kind, direction="R-up", client="operator", age=0, seq=None):
            nonlocal sequence
            sequence += 1
            policy.observe(transport.get_state()[0])
            return policy.request(client, sequence if seq is None else seq,
                                  time.time() - age, kind, direction, dispatch, enabled=True)

        assert not send("start").accepted, "Bridge startup must require explicit Stop"
        assert not dispatches, "Rejected startup command was forwarded"
        assert send("stop").accepted
        assert send("start").accepted
        for _ in range(8):
            time.sleep(.05)
            assert send("hold").accepted
        moved = request("/state")[1]
        assert moved["sequence"] > initial["sequence"], repr(moved)
        assert feedback_check(moved)[0]
        before = len(dispatches)
        assert not send("hold", client="competitor").accepted
        assert not send("hold", direction="R-down").accepted
        assert not send("hold", seq=1).accepted
        assert not send("hold", age=1).accepted
        assert not send("start").accepted
        assert len(dispatches) == before, "Rejected requests reached the simulator"
        assert send("stop").accepted
        assert request("/state")[1]["state"] == "Disarmed"
        print("PASS actual motion, explicit startup acknowledgement and command rejection", flush=True)

        assert send("start").accepted
        for _ in range(5):
            time.sleep(.05)
            assert send("hold").accepted
        # No bridge-generated Hold or Stop: the C++ monitor must latch itself.
        time.sleep(.3)
        latched = request("/state")[1]
        assert latched["state"] == "AvoidanceLatched" and latched["speed_dps"] == 0, repr(latched)
        assert not send("hold").accepted
        assert not send("start").accepted
        assert request("/state")[1]["sequence"] == latched["sequence"]
        assert request("/state")[1]["state"] == "AvoidanceLatched"
        assert send("stop").accepted
        assert send("start").accepted
        assert send("stop").accepted
        print("PASS lost-input watchdog, preserved latch and explicit rearm", flush=True)

        # A lost bridge connection must invalidate local ownership without
        # acknowledging any remote state; recovering transport does not rearm.
        policy.observe(None)
        before = len(dispatches)
        assert not send("start").accepted
        assert len(dispatches) == before
        assert send("stop").accepted
        stale = dict(request("/state")[1], pose_age_ms=1000)
        policy.observe(stale)
        sequence += 1
        rejected = policy.request("operator", sequence, time.time(), "start", "R-up",
                                  dispatch, enabled=True)
        assert not rejected.accepted
        sequence += 1
        stopped = policy.request("operator", sequence, time.time(), "stop", "",
                                 dispatch, enabled=True)
        assert stopped.accepted, "Stale pose must not prevent explicit Stop"
        print("PASS disconnect/stale feedback blocks Start but permits explicit Stop", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/pcan_demo")
    run(parser.parse_args().binary)
