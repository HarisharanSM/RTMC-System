"""Owned-process ROS 2 smoke check for simulation, service, and telemetry bag."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import sqlite3
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rtmc_interfaces.srv import Jog
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.serialization import deserialize_message

from .smoke_checks import diagnostic_details, healthy_diagnostic


URL = 'http://127.0.0.1:8082'
HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def state():
    with HTTP.open(URL + '/state', timeout=.3) as reply:
        return json.load(reply)


def port_free():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', 8082))
            return True
        except OSError:
            return False


def wait_for(predicate, label, seconds=8, process=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if process is not None and process.poll() is not None:
            raise AssertionError(label + ': owned process exited with ' + str(process.returncode))
        try:
            result = predicate()
            if result:
                return result
        except (OSError, urllib.error.URLError):
            pass
        time.sleep(.05)
    raise AssertionError('timed out waiting for ' + label)


def start_process(command, log_path):
    log = open(log_path, 'w')
    try:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
    except BaseException:
        log.close()
        raise
    return proc, log


def stop_process(proc, log):
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=2)
    if log is not None:
        log.close()


class Probe:
    def __init__(self):
        self.node = rclpy.create_node('rtmc_smoke_probe')
        self.service = self.node.create_client(Jog, '/rtmc/jog')
        self.states = []
        self.joints = []
        self.diagnostics = []
        self.node.create_subscription(String, '/rtmc/state',
                                      lambda m: self.states.append(json.loads(m.data)), 10)
        self.node.create_subscription(JointState, '/rtmc/joint_states',
                                      lambda m: self.joints.append(m), 10)
        self.node.create_subscription(DiagnosticArray, '/rtmc/diagnostics',
                                      lambda m: self.diagnostics.append(m), 10)
        self.sequence = 0

    def spin(self, seconds=.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(.05, end - time.monotonic())))

    def jog(self, command, direction='R-up', *, client='smoke',
            sequence=None, stamp_offset=0.0, accepted=True):
        if sequence is None:
            self.sequence += 1
            sequence = self.sequence
        req = Jog.Request()
        instant_ns = time.time_ns() + int(stamp_offset * 1e9)
        req.stamp.sec = instant_ns // 1_000_000_000
        req.stamp.nanosec = instant_ns % 1_000_000_000
        req.client_id = client
        req.sequence = sequence
        req.command = command
        req.direction = direction
        future = self.service.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1)
        if not future.done() or future.result() is None:
            raise AssertionError(command + ' service unavailable')
        result = future.result()
        if result.accepted != accepted:
            raise AssertionError('%s accepted=%s expected=%s: %s' %
                                 (command, result.accepted, accepted, result.message))
        return result

    def close(self):
        self.node.destroy_node()


def inspect_bag(bag):
    """Decode stored SQLite messages, not just rosbag's metadata topic list."""
    recorded = {'/rtmc/state': [], '/rtmc/joint_states': [], '/rtmc/diagnostics': []}
    types = {'/rtmc/state': String, '/rtmc/joint_states': JointState,
             '/rtmc/diagnostics': DiagnosticArray}
    for database in bag.glob('*.db3'):
        with sqlite3.connect('file:' + str(database.resolve()) + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('SELECT id, name FROM topics'))
            assert set(topics.values()) <= set(recorded), 'bag contains an unexpected topic'
            for topic_id, data in db.execute('SELECT topic_id, data FROM messages ORDER BY timestamp'):
                topic = topics[topic_id]
                message = deserialize_message(data, types[topic])
                recorded[topic].append(json.loads(message.data) if topic == '/rtmc/state' else message)
    assert all(recorded.values()), 'one or more telemetry topics have no stored samples'
    for msg in recorded['/rtmc/joint_states']:
        assert msg.name == ['A1', 'A2', 'A3', 'A4', 'A5']
        assert len(msg.position) == 5 and all(math.isfinite(x) for x in msg.position)
    assert any(all(abs(a - math.radians(b)) < 1e-10 for a, b in zip(j.position, s['axles_deg']))
               for j in recorded['/rtmc/joint_states'] for s in recorded['/rtmc/state']), \
        'bag joint radians do not match any recorded CAN pose'
    return recorded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--evidence-dir', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file() or not args.assets.is_dir():
        parser.error('binary and assets must exist')
    if not port_free():
        raise SystemExit('Refusing to run: port 8082 already has a listener')
    evidence = args.evidence_dir or Path(tempfile.mkdtemp(prefix='rtmc-ros2-smoke-'))
    evidence.mkdir(parents=True, exist_ok=True)
    if any(evidence.iterdir()):
        parser.error('evidence directory must be empty; preserve earlier smoke results')
    sim = bridge = recorder = player = None
    sim_log = bridge_log = record_log = play_log = None
    report = {'status': 'incomplete', 'stages': [], 'evidence_dir': str(evidence.resolve())}
    rclpy.init()
    probe = Probe()
    try:
        probe.spin(.3)
        if probe.service.service_is_ready():
            raise RuntimeError('Refusing to run: /rtmc/jog already exists in this ROS domain')
        sim, sim_log = start_process([str(args.binary.resolve()), '--assets',
                                      str(args.assets.resolve()), '--command-source', 'ros2'],
                                     evidence / 'simulator.log')
        wait_for(lambda: state().get('command_source') == 'ros2', 'owned ROS simulator', process=sim)
        report['stages'].append('owned simulator in ros2 mode')
        bridge, bridge_log = start_process(['ros2', 'launch', 'rtmc_ros2', 'bridge.launch.py',
                                            'enable_commands:=true'], evidence / 'bridge.log')
        wait_for(lambda: probe.service.wait_for_service(timeout_sec=.1), '/rtmc/jog discovery', process=bridge)
        wait_for(lambda: (probe.spin(.1) or len(probe.states) > 0 and
                          len(probe.joints) > 0 and len(probe.diagnostics) > 0),
                 'ROS state/joints/diagnostics')
        assert probe.joints[-1].name == ['A1', 'A2', 'A3', 'A4', 'A5']
        assert len(probe.joints[-1].position) == 5
        assert not probe.joints[-1].velocity and not probe.joints[-1].effort
        assert all(abs(a - math.radians(b)) < 1e-10
                   for a, b in zip(probe.joints[-1].position, state()['axles_deg']))
        assert probe.states[-1].get('simulation_only') is True
        try:
            wait_for(lambda: (probe.spin(.05) or healthy_diagnostic(
                probe.diagnostics[-1] if probe.diagnostics else None, DiagnosticStatus.OK)),
                'fresh healthy bridge diagnostics', process=bridge)
        except AssertionError as exc:
            detail = diagnostic_details(probe.diagnostics[-1] if probe.diagnostics else None)
            raise AssertionError(str(exc) + '; latest ' + detail) from exc
        report['stages'].append('service and telemetry discovery; five axes in radians')

        recorder, record_log = start_process([
            'ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(evidence / 'telemetry_bag'),
            '/rtmc/state', '/rtmc/joint_states', '/rtmc/diagnostics'],
            evidence / 'bag_record.log')
        time.sleep(.5)
        probe.jog('start', accepted=False)  # restart requires acknowledgement
        probe.jog('stop')
        before = state()['sequence']
        probe.jog('start')
        moved = False
        for _ in range(20):
            time.sleep(.05)
            probe.jog('hold')
            if state()['sequence'] > before:
                moved = True
                break
        assert moved, 'accepted jog did not advance the actual CAN pose'
        probe.jog('hold', client='other', sequence=1, accepted=False)
        probe.jog('hold', sequence=probe.sequence, accepted=False)
        probe.jog('hold', stamp_offset=-.25, accepted=False)
        probe.jog('hold', stamp_offset=1.0, accepted=False)
        probe.jog('hold', direction='invalid', accepted=False)
        time.sleep(.23)
        wait_for(lambda: state()['state'] == 'AvoidanceLatched', 'C++ renewal watchdog')
        stopped_sequence = state()['sequence']
        assert state()['speed_dps'] == 0
        probe.jog('start', accepted=False)
        assert state()['sequence'] == stopped_sequence
        probe.jog('stop')
        wait_for(lambda: state()['state'] == 'Disarmed', 'explicit Stop acknowledgement')
        report['stages'].append('motion, owner/sequence/timestamp rejection, watchdog and Stop')

        # The bridge must not manufacture Stop during a simulator disconnect.
        stop_process(sim, sim_log)
        sim = sim_log = None
        wait_for(lambda: (probe.spin(.1) or any(
            item.key == 'connection' and item.value == 'disconnected'
            for item in probe.diagnostics[-1].status[0].values)),
                 'disconnect diagnostics')
        probe.jog('start', accepted=False)
        sim, sim_log = start_process([str(args.binary.resolve()), '--assets',
                                      str(args.assets.resolve()), '--command-source', 'ros2'],
                                     evidence / 'simulator_restart.log')
        wait_for(lambda: state().get('command_source') == 'ros2', 'simulator reconnect', process=sim)
        time.sleep(.2)
        probe.jog('start', accepted=False)
        probe.jog('stop')
        report['stages'].append('simulator disconnection/reconnect requires Stop')

        # Kill an actively controlling bridge; the C++ watchdog must stop on
        # its own. No Stop is sent during failure or shutdown.
        before = state()['sequence']
        probe.jog('start')
        for _ in range(8):
            time.sleep(.05)
            probe.jog('hold')
        assert state()['sequence'] > before
        stop_process(bridge, bridge_log)
        bridge = bridge_log = None
        wait_for(lambda: state()['state'] == 'AvoidanceLatched', 'watchdog after bridge termination')
        assert state()['speed_dps'] == 0
        wait_for(lambda: not probe.service.service_is_ready(), 'old bridge discovery removal')
        bridge, bridge_log = start_process(['ros2', 'launch', 'rtmc_ros2', 'bridge.launch.py',
                                            'enable_commands:=true'],
                                           evidence / 'bridge_restart.log')
        wait_for(lambda: probe.service.wait_for_service(timeout_sec=.1), 'bridge restart', process=bridge)
        time.sleep(.2)
        probe.jog('start', accepted=False)
        probe.jog('stop')
        report['stages'].append('bridge termination while moving stops drive; restart requires Stop')

        probe.spin(.5)
        stop_process(recorder, record_log)
        recorder = record_log = None
        bag = evidence / 'telemetry_bag'
        assert bag.joinpath('metadata.yaml').is_file(), 'rosbag metadata missing'
        info = subprocess.run(['ros2', 'bag', 'info', str(bag)], text=True,
                              capture_output=True, timeout=10, check=True)
        (evidence / 'bag_info.txt').write_text(info.stdout)
        for topic in ('/rtmc/state', '/rtmc/joint_states', '/rtmc/diagnostics'):
            assert topic in info.stdout, 'bag topic missing: ' + topic
        recorded = inspect_bag(bag)
        report['bag_messages'] = {topic: len(values) for topic, values in recorded.items()}
        stop_process(bridge, bridge_log)
        bridge = bridge_log = None
        stop_process(sim, sim_log)
        sim = sim_log = None
        probe.spin(.3)  # drain already-received live samples before replay
        probe.states.clear()
        probe.joints.clear()
        probe.diagnostics.clear()
        # Allow DDS discovery before the first stored sample. A short bag can
        # otherwise finish before this already-created probe matches rosbag.
        player, play_log = start_process(['ros2', 'bag', 'play', str(bag),
                                          '--delay', '3.0'], evidence / 'bag_play.log')
        try:
            wait_for(lambda: (probe.spin(.1) or
                              (probe.states and probe.joints and probe.diagnostics)),
                     'telemetry-only rosbag replay', seconds=12)
        except AssertionError as exc:
            counts = {'state': len(probe.states), 'joints': len(probe.joints),
                      'diagnostics': len(probe.diagnostics)}
            raise AssertionError('%s; received=%r; recorded=%r; player_exit=%r; '
                'player_log=%s' % (exc, counts, report['bag_messages'], player.poll(),
                                   evidence / 'bag_play.log')) from exc
        assert probe.states[0] in recorded['/rtmc/state'], 'replayed state differs from stored state'
        assert any(list(probe.joints[0].position) == list(msg.position)
                   for msg in recorded['/rtmc/joint_states']), 'replayed joints differ from stored joints'
        stop_process(player, play_log)
        player = play_log = None
        report['stages'].append('telemetry bag record and replay with control offline')
        report['status'] = 'passed'
        print(json.dumps(report, indent=2))
    except BaseException as exc:
        report['status'] = 'failed'
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        raise
    finally:
        for proc, log in ((player, play_log), (recorder, record_log),
                          (bridge, bridge_log), (sim, sim_log)):
            stop_process(proc, log)
        probe.close()
        rclpy.try_shutdown()
        (evidence / 'smoke_result.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
