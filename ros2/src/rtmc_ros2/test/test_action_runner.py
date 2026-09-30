import unittest

from rtmc_ros2.action_runner import ActionJogRunner
from rtmc_ros2.policy import JogPolicy


def state(lifecycle='Disarmed', speed=0, moved=False):
    return dict(simulation_only=True, pose_source='drive_can_feedback',
        feedback_valid=True, pose_age_ms=2, can_feedback_frames=7,
        pose_sequence=2 if moved else 1, sequence=2 if moved else 1,
        axles_deg=[1 if moved else 0, 0, 0, 0, 90],
        protocol_version=4, collision_enabled=True, command_source='ros2',
        state=lifecycle, speed_dps=speed)


class ActionRunnerTest(unittest.TestCase):
    def setUp(self):
        self.policy = JogPolicy()
        self.policy.observe(state())
        self.calls = []
        def dispatch(command, direction):
            self.calls.append(command)
            return dict(accepted=True, input_sequence=len(self.calls),
                        session=0 if command == 'stop' else 2)
        self.dispatch = dispatch
        self.policy.request('operator', 1, 100, 'stop', '', dispatch,
                            enabled=True, now_wall=100, now_mono=1)

    def runner(self, duration=.2):
        return ActionJogRunner(self.policy, 'A3-right', duration, self.dispatch, enabled=True)

    def step(self, runner, when, cancel=False):
        return runner.step(now_mono=when, now_wall=100+when, cancel=cancel)

    def test_reservation_blocks_competing_service_and_no_catchup(self):
        runner = self.runner()
        self.assertFalse(self.policy.request('other', 1, 100, 'start', 'R-up',
            self.dispatch, enabled=True, now_wall=100, now_mono=1).accepted)
        self.step(runner, 1)
        self.policy.observe(state('Running', 10, moved=True))
        self.step(runner, 1.06)
        self.assertEqual(self.calls, ['stop', 'start', 'hold'])
        self.step(runner, 1.12)
        self.assertEqual(self.calls, ['stop', 'start', 'hold', 'hold'])
        outcome = self.step(runner, 1.21)
        self.assertIsNone(outcome)
        self.policy.observe(state(moved=True))
        outcome = self.step(runner, 1.26)
        self.assertTrue(outcome.completed)
        self.assertTrue(outcome.motion_observed)
        self.assertFalse(outcome.explicit_stop_required)
        self.assertEqual(self.calls[-1], 'stop')

    def test_late_tick_aborts_without_stop(self):
        runner = self.runner()
        self.step(runner, 1)
        self.policy.observe(state('Running', 10))
        outcome = self.step(runner, 1.16)
        self.assertFalse(outcome.completed)
        self.assertTrue(outcome.explicit_stop_required)
        self.assertIn('policy requires explicit Stop', outcome.message)
        self.assertEqual(self.calls, ['stop', 'start'])

    def test_cancel_and_explicit_service_stop(self):
        runner = self.runner()
        self.step(runner, 1)
        self.policy.observe(state('Running', 10))
        outcome = self.step(runner, 1.05, cancel=True)
        self.assertIsNone(outcome)
        self.policy.observe(state())
        outcome = self.step(runner, 1.10)
        self.assertTrue(outcome.cancelled)
        self.assertFalse(outcome.explicit_stop_required)
        self.assertEqual(self.calls[-1], 'stop')
        self.policy.observe(state())
        runner = self.runner()
        self.step(runner, 2)
        self.policy.observe(state('Running', 10))
        self.assertTrue(self.policy.request('operator', 2, 102, 'stop', '', self.dispatch,
            enabled=True, now_wall=102, now_mono=2.03).accepted)
        self.policy.observe(state())
        self.assertTrue(self.policy.request('new-client', 1, 102.04, 'start', 'R-up',
            self.dispatch, enabled=True, now_wall=102.04, now_mono=2.04).accepted)
        before = len(self.calls)
        outcome = self.step(runner, 2.05)
        self.assertFalse(outcome.completed)
        self.assertEqual(len(self.calls), before)
        self.assertEqual(self.policy.active[:2], ('new-client', 'R-up'))

    def test_uncertain_hold_and_latch_never_stop(self):
        runner = self.runner()
        self.step(runner, 1)
        self.policy.observe(state('AvoidanceLatched'))
        outcome = self.step(runner, 1.05)
        self.assertTrue(outcome.explicit_stop_required)
        self.assertIn('policy requires explicit Stop', outcome.message)
        self.assertEqual(self.calls, ['stop', 'start'])

    def test_uncertain_hold_and_stop_never_retry_or_acknowledge(self):
        for failing_command in ('hold', 'stop'):
            for failure in ('exception', 'rejection'):
              with self.subTest(failing_command=failing_command, failure=failure):
                self.setUp()
                runner = self.runner(.2)
                self.step(runner, 1)
                self.policy.observe(state('Running', 10))
                def lost(command, direction):
                    self.calls.append(command)
                    if failure == 'exception':
                        raise TimeoutError('receipt lost')
                    return {'accepted': False}
                runner.dispatch = lost
                when = 1.06 if failing_command == 'hold' else 1.21
                outcome = self.step(runner, when)
                self.assertFalse(outcome.completed)
                self.assertTrue(outcome.explicit_stop_required)
                self.assertTrue(self.policy.needs_stop)
                self.assertEqual(self.calls.count(failing_command), 1)
                self.step(runner, when + .05)
                self.assertEqual(self.calls.count(failing_command), 1)
                with self.assertRaises(ValueError):
                    self.runner()
                self.assertTrue(self.policy.request('operator', 2, 103, 'stop', '',
                    self.dispatch, enabled=True, now_wall=103, now_mono=3).accepted)
                self.policy.observe(state())
                recovered = self.runner()
                recovered.step(now_mono=3.01, now_wall=103.01, cancel=True)

    def test_stale_or_disconnect_before_cancel_never_stops(self):
        for observation in (state('Running', 10) | {'pose_age_ms': 300}, None):
            with self.subTest(observation=observation):
                self.setUp()
                runner = self.runner()
                self.step(runner, 1)
                self.policy.observe(observation)
                outcome = self.step(runner, 1.05, cancel=True)
                self.assertFalse(outcome.completed)
                self.assertTrue(outcome.explicit_stop_required)
                self.assertIn('explicit Stop required', outcome.message)
                self.assertEqual(self.calls, ['stop', 'start'])

    def test_stop_confirmation_timeout_requires_explicit_stop(self):
        runner = self.runner(.2)
        self.step(runner, 1)
        self.policy.observe(state('Running', 10))
        self.step(runner, 1.06)
        self.step(runner, 1.12)
        self.assertIsNone(self.step(runner, 1.21))
        self.assertEqual(self.calls[-1], 'stop')
        before = len(self.calls)
        outcome = self.step(runner, 1.37)
        self.assertFalse(outcome.completed)
        self.assertTrue(outcome.explicit_stop_required)
        self.assertTrue(self.policy.needs_stop)
        self.assertEqual(len(self.calls), before)

    def test_stop_confirmation_reserves_until_feedback_or_external_stop(self):
        runner = self.runner(.2)
        self.step(runner, 1)
        self.policy.observe(state('Running', 10))
        self.step(runner, 1.06)
        self.step(runner, 1.12)
        self.assertIsNone(self.step(runner, 1.21))
        self.policy.observe(state())
        self.assertFalse(self.policy.request('new-client', 1, 101.22, 'start', 'R-up',
            self.dispatch, enabled=True, now_wall=101.22, now_mono=1.22).accepted)
        self.assertTrue(self.policy.request('operator', 2, 101.23, 'stop', '',
            self.dispatch, enabled=True, now_wall=101.23, now_mono=1.23).accepted)
        self.policy.observe(state())
        self.assertTrue(self.policy.request('new-client', 2, 101.24, 'start', 'R-up',
            self.dispatch, enabled=True, now_wall=101.24, now_mono=1.24).accepted)
        before = len(self.calls)
        outcome = self.step(runner, 1.25)
        self.assertFalse(outcome.completed)
        self.assertFalse(outcome.explicit_stop_required)
        self.assertEqual(len(self.calls), before)
        self.assertEqual(self.policy.active[:2], ('new-client', 'R-up'))

    def test_bounds_and_prior_stop(self):
        for duration in (0, .099, 5.001, float('nan')):
            with self.assertRaises(ValueError):
                self.runner(duration)
        self.assertFalse(self.policy.needs_stop)
        self.policy.needs_stop = True
        with self.assertRaises(ValueError):
            self.runner()


if __name__ == '__main__':
    unittest.main()
