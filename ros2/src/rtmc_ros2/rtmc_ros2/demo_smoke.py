"""Owned integrated Jazzy demo: topics, service, action and live URDF transforms."""

import argparse
import json
import math
from pathlib import Path
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.time import Time
from rtmc_interfaces.action import JogFor
from tf2_ros import Buffer, TransformListener

from .smoke import Probe, port_free, start_process, state, stop_process, wait_for


def completed_future(probe, future, label, timeout=8):
    wait_for(lambda: (probe.spin(.02) or future.done()), label, seconds=timeout)
    result = future.result()
    assert result is not None, label + ' returned no result'
    return result


def check_transform(probe, buffer):
    probe.spin(.2)
    sample = probe.joints[-1]
    instant = Time.from_msg(sample.header.stamp)
    wait_for(lambda: (probe.spin(.02) or buffer.can_transform(
        'rtmc_patient', 'imaging_center', instant)), 'TF at joint feedback timestamp')
    transform = buffer.lookup_transform('rtmc_patient', 'imaging_center', instant).transform
    a1, a2, a3, _, _ = sample.position
    heading = a1 + a2 + a3
    expected = (-1.40 + .75 * math.cos(a1) + math.cos(a1 + a2) + 1.15 * math.cos(heading),
                .75 * math.sin(a1) + math.sin(a1 + a2) + 1.15 * math.sin(heading), 1.20)
    actual = (transform.translation.x, transform.translation.y, transform.translation.z)
    assert max(abs(a-b) for a, b in zip(actual, expected)) < 1e-8, (actual, expected)
    q = transform.rotation
    expected_q = (0., 0., math.sin(heading / 2), math.cos(heading / 2))
    actual_q = (q.x, q.y, q.z, q.w)
    assert min(max(abs(a-b) for a, b in zip(actual_q, expected_q)),
               max(abs(a+b) for a, b in zip(actual_q, expected_q))) < 1e-8
    return {'position_m': actual, 'joint_stamp_sec': sample.header.stamp.sec,
            'joint_stamp_nanosec': sample.header.stamp.nanosec}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--evidence-dir', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file() or not args.assets.is_dir():
        parser.error('binary and assets must exist')
    if not port_free():
        parser.error('port 8082 is occupied; this smoke only owns its own processes')
    evidence = args.evidence_dir or Path(tempfile.mkdtemp(prefix='rtmc-demo-'))
    evidence.mkdir(parents=True, exist_ok=True)
    if any(evidence.iterdir()):
        parser.error('evidence directory must be empty')
    report = {'status': 'incomplete', 'stages': [], 'simulation_only': True}
    demo = log = None
    rclpy.init()
    probe = Probe()
    action = ActionClient(probe.node, JogFor, '/rtmc/jog_for')
    buffer = Buffer()
    listener = TransformListener(buffer, probe.node)
    feedback = []

    def goal(duration=.5, accepted=True):
        request = JogFor.Goal()
        request.direction = 'R-up'
        request.duration_sec = duration
        handle = completed_future(probe, action.send_goal_async(
            request, feedback_callback=lambda message: feedback.append(message.feedback)), 'action goal')
        assert handle.accepted == accepted, 'unexpected goal acceptance'
        return handle

    try:
        probe.spin(.4)
        if probe.service.service_is_ready() or action.server_is_ready():
            raise RuntimeError('existing RTMC service/action in ROS domain; refusing to command it')
        demo, log = start_process(['ros2', 'launch', 'rtmc_ros2', 'demo.launch.py',
            'simulator_binary:=' + str(args.binary.resolve()),
            'assets:=' + str(args.assets.resolve()), 'enable_commands:=true'], evidence / 'demo.log')
        wait_for(lambda: probe.service.wait_for_service(timeout_sec=.1), 'service discovery', process=demo)
        wait_for(lambda: action.wait_for_server(timeout_sec=.1), 'action discovery', process=demo)
        wait_for(lambda: (probe.spin(.05) or bool(probe.states and probe.joints and probe.diagnostics)),
                 'all telemetry topics', process=demo)
        assert probe.joints[-1].name == ['A1', 'A2', 'A3', 'A4', 'A5']
        assert not probe.joints[-1].velocity and not probe.joints[-1].effort
        report['stages'].append('topics, service and action discovered')
        report['home_tf'] = check_transform(probe, buffer)
        goal(accepted=False)  # Startup must not be automatically acknowledged.
        probe.jog('stop')
        goal(duration=float('nan'), accepted=False)
        goal(duration=6., accepted=False)

        before = state()['axles_deg']
        probe.jog('start')
        for _ in range(8):
            probe.spin(.05)
            probe.jog('hold')
        probe.jog('stop')
        assert state()['state'] == 'Disarmed'
        assert state()['axles_deg'] != before, 'service jog produced no actual movement'
        report['service_tf'] = check_transform(probe, buffer)
        report['stages'].append('service jog moves coherent CAN feedback and TF')

        handle = goal()
        result = completed_future(probe, handle.get_result_async(), 'normal action completion')
        assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.completed
        assert result.result.motion_observed and not result.result.explicit_stop_required
        assert state()['state'] == 'Disarmed' and state()['speed_dps'] == 0
        assert feedback and any(f.feedback_valid for f in feedback)
        report['stages'].append('action feedback and finite completion')

        feedback.clear()
        handle = goal(duration=3.)
        wait_for(lambda: (probe.spin(.02) or any(f.controller_state == 'Running' for f in feedback)),
                 'action running before cancellation')
        goal(accepted=False)
        probe.jog('hold', client='competitor', sequence=1, accepted=False)
        response = completed_future(probe, handle.cancel_goal_async(), 'cancel request')
        assert response.goals_canceling, 'cancellation not accepted'
        result = completed_future(probe, handle.get_result_async(), 'cancelled result')
        assert result.status == GoalStatus.STATUS_CANCELED and not result.result.completed
        assert not result.result.explicit_stop_required
        assert state()['state'] == 'Disarmed' and state()['speed_dps'] == 0
        report['stages'].append('concurrent goal/hold rejected; cancellation stops owned motion')

        handle = goal(duration=3.)
        probe.spin(.15)
        probe.jog('stop')
        result = completed_future(probe, handle.get_result_async(), 'service Stop interrupts action')
        assert result.status == GoalStatus.STATUS_ABORTED
        snapshot = state()['axles_deg']
        probe.spin(.25)
        assert state()['state'] == 'Disarmed' and state()['axles_deg'] == snapshot
        report['stages'].append('explicit service Stop prevents later action motion')

        # An input stream ending without Stop must preserve the watchdog latch.
        probe.jog('stop')
        probe.jog('start')
        for _ in range(4):
            probe.spin(.05)
            probe.jog('hold')
        wait_for(lambda: (probe.spin(.05) or state()['state'] == 'AvoidanceLatched'), 'input-loss watchdog')
        assert state()['speed_dps'] == 0
        goal(accepted=False)
        assert state()['state'] == 'AvoidanceLatched'
        probe.jog('stop')
        report['final_tf'] = check_transform(probe, buffer)
        report['stages'].append('lost input latches; action cannot acknowledge; explicit Stop recovers')
        report['status'] = 'passed'
        print(json.dumps(report, indent=2))
    except BaseException as exc:
        report['status'] = 'failed'
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        raise
    finally:
        stop_process(demo, log)
        action.destroy()
        listener.unregister()
        probe.close()
        rclpy.try_shutdown()
        (evidence / 'demo_result.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
