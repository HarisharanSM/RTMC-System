#!/usr/bin/env python3
"""Exercise the ROS-free action runner against an owned C++ simulator."""

import argparse
from pathlib import Path
import sys
import time

from test_command_source import ROOT, request, simulator

sys.path.insert(0, str(ROOT / 'ros2/src/rtmc_ros2'))
from rtmc_ros2.action_runner import ActionJogRunner
from rtmc_ros2.policy import JogPolicy
from rtmc_ros2.transport import SimulatorClient


def run(binary):
    with simulator(binary, 'ros2') as initial:
        transport = SimulatorClient()
        policy = JogPolicy()
        policy.observe(initial)
        try:
            ActionJogRunner(policy, 'R-up', .5, transport.dispatch, enabled=True)
            raise AssertionError('startup action accepted without explicit Stop')
        except ValueError:
            pass
        stopped = policy.request('operator', 1, time.time(), 'stop', '',
            transport.dispatch, enabled=True)
        assert stopped.accepted, stopped
        policy.observe(transport.get_state()[0])
        runner = ActionJogRunner(policy, 'R-up', .6, transport.dispatch, enabled=True)
        assert not policy.request('other', 1, time.time(), 'start', 'R-up',
            transport.dispatch, enabled=True).accepted
        before = request('/state')[1]['sequence']
        outcome = None
        while outcome is None:
            policy.observe(transport.get_state()[0])
            outcome = runner.step()
            if outcome is None:
                time.sleep(.05)
        assert outcome.completed and outcome.motion_observed and not outcome.explicit_stop_required, (outcome, policy.state, policy.active)
        after = request('/state')[1]
        assert after['sequence'] > before and after['state'] == 'Disarmed', after
        print('PASS bounded action drove real CAN feedback and dispatched Stop', flush=True)

        policy.observe(after)
        runner = ActionJogRunner(policy, 'R-up', 1., transport.dispatch, enabled=True)
        runner.step()
        time.sleep(.06)
        policy.observe(transport.get_state()[0])
        cancelled = runner.step(cancel=True)
        assert cancelled is None
        policy.observe(transport.get_state()[0])
        cancelled = runner.step()
        assert cancelled.cancelled and not cancelled.explicit_stop_required, cancelled
        assert request('/state')[1]['state'] == 'Disarmed'
        print('PASS healthy cancellation dispatched Stop', flush=True)

        policy.observe(transport.get_state()[0])
        runner = ActionJogRunner(policy, 'R-up', 1., transport.dispatch, enabled=True)
        runner.step()
        policy.observe(transport.get_state()[0])
        explicit = policy.request('operator', 2, time.time(), 'stop', '',
            transport.dispatch, enabled=True)
        assert explicit.accepted, explicit
        policy.observe(transport.get_state()[0])
        newcomer = policy.request('new-client', 1, time.time(), 'start', 'R-up',
            transport.dispatch, enabled=True)
        assert newcomer.accepted, newcomer
        before = request('/state')[1]['sequence']
        outcome = runner.step()
        assert not outcome.completed and not outcome.explicit_stop_required, outcome
        assert request('/state')[1]['sequence'] == before
        assert policy.active[:2] == ('new-client', 'R-up')
        assert policy.request('new-client', 2, time.time(), 'stop', '',
            transport.dispatch, enabled=True).accepted
        print('PASS service Stop and immediate new owner prevent old action commands', flush=True)

        policy.observe(transport.get_state()[0])
        runner = ActionJogRunner(policy, 'R-up', 1., transport.dispatch, enabled=True)
        runner.step()
        time.sleep(.3)
        latched = request('/state')[1]
        assert latched['state'] == 'AvoidanceLatched' and latched['speed_dps'] == 0, latched
        policy.observe(latched)
        before = latched['sequence']
        outcome = runner.step()
        assert not outcome.completed and outcome.explicit_stop_required, outcome
        assert request('/state')[1]['sequence'] == before
        try:
            ActionJogRunner(policy, 'R-up', .5, transport.dispatch, enabled=True)
            raise AssertionError('latched action accepted')
        except ValueError:
            pass
        print('PASS lost input retained latch and required explicit Stop', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=ROOT / 'build/pcan_demo')
    run(parser.parse_args().binary)
