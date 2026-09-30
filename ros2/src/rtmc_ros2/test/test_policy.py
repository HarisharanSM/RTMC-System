import math
import unittest

from rtmc_ros2.policy import JogPolicy, feedback_check
from rtmc_ros2.transport import SimulatorClient


def state(**updates):
    value = dict(simulation_only=True, pose_source='drive_can_feedback',
                 feedback_valid=True, pose_age_ms=2, can_feedback_frames=7,
                 pose_sequence=1, sequence=1, axles_deg=[0, 0, 0, 0, 90],
                 protocol_version=4, collision_enabled=True,
                 command_source='ros2', state='Disarmed', reason='')
    value.update(updates)
    return value


class PolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = JogPolicy(max_clients=2)
        self.policy.observe(state())
        self.calls = []

    def dispatch(self, command, direction):
        self.calls.append((command, direction))
        return {'accepted': True, 'input_sequence': len(self.calls),
                'session': 0 if command == 'stop' else 17}

    def call(self, sequence, command, direction='A3-right', client='a',
             stamp=100.0, mono=1.0, dispatch=None):
        return self.policy.request(client, sequence, stamp, command, direction,
                                   dispatch or self.dispatch, enabled=True,
                                   now_wall=100.0, now_mono=mono)

    def test_stop_is_explicit_and_hold_is_single_forward(self):
        self.assertFalse(self.call(1, 'start').accepted)
        self.assertEqual(self.calls, [])
        self.assertTrue(self.call(2, 'stop').accepted)
        self.assertTrue(self.call(3, 'start').accepted)
        self.policy.observe(state(state='Running'))
        self.assertFalse(self.call(4, 'hold', client='b', mono=1.04).accepted)
        self.assertFalse(self.call(5, 'hold', direction='R-up', mono=1.04).accepted)
        self.assertTrue(self.call(6, 'hold', mono=1.04).accepted)
        self.assertEqual(self.calls, [('stop', 'A3-right'), ('start', 'A3-right'),
                                      ('hold', 'A3-right')])
        self.assertTrue(self.call(5, 'stop', client='b', mono=1.05).accepted)
        self.assertIsNone(self.policy.active)

    def test_timeout_and_uncertain_command_require_stop(self):
        self.call(1, 'stop')
        self.call(2, 'start')
        self.policy.observe(state(state='Running'))
        self.assertFalse(self.call(3, 'hold', mono=1.151).accepted)
        self.assertTrue(self.policy.needs_stop)
        self.assertFalse(self.call(4, 'start', mono=1.152).accepted)
        self.assertTrue(self.call(5, 'stop', mono=1.153).accepted)
        self.policy.observe(state())
        def broken(*_):
            raise TimeoutError('timeout')
        result = self.call(6, 'start', mono=1.154, dispatch=broken)
        self.assertFalse(result.accepted)
        self.assertIn('outcome uncertain', result.message)
        self.assertIn('explicit Stop', self.call(7, 'start', mono=1.155).message)
        self.assertTrue(self.policy.needs_stop)

    def test_timestamp_sequence_and_client_bound(self):
        self.assertFalse(self.call(1, 'stop', stamp=99.8).accepted)
        self.assertFalse(self.call(1, 'stop', stamp=100.051).accepted)
        self.assertTrue(self.call(1, 'stop').accepted)
        self.assertFalse(self.call(1, 'stop').accepted)
        self.assertTrue(self.call(1, 'stop', client='b').accepted)
        self.assertFalse(self.call(1, 'stop', client='c').accepted)
        self.assertEqual(len(self.calls), 2)

    def test_stale_feedback_and_reconnect_never_acknowledge_latch(self):
        self.call(1, 'stop')
        self.call(2, 'start')
        self.policy.observe(state(state='Running'))
        self.policy.observe(state(pose_age_ms=251))
        self.assertFalse(self.call(3, 'hold').accepted)
        self.policy.observe(None)
        self.policy.observe(state())
        self.assertFalse(self.call(4, 'start').accepted)
        self.assertTrue(self.call(5, 'stop').accepted)
        self.assertTrue(self.call(6, 'start').accepted)
        self.policy.observe(state(state='AvoidanceLatched', reason='watchdog'))
        self.assertFalse(self.call(7, 'hold').accepted)
        self.assertTrue(self.policy.needs_stop)

    def test_feedback_requires_finite_coherent_fresh_axes(self):
        ok, _, axes = feedback_check(state())
        self.assertTrue(ok)
        self.assertAlmostEqual(axes[4], math.pi / 2)
        for broken in (state(axles_deg=[0, 0, 0]),
                       state(axles_deg=[0, 0, float('nan'), 0, 0]),
                       state(pose_age_ms=249),
                       state(feedback_valid=False),
                       state(pose_source='other'),
                       state(sequence=2),
                       state(axles_deg=[0, 0, 0, 0, 361]),
                       state(protocol_version=3)):
            elapsed = 2 if broken['pose_age_ms'] == 249 else 0
            self.assertFalse(feedback_check(broken, elapsed)[0])

    def test_browser_mode_and_disabled_control_reject(self):
        self.policy.observe(state(command_source='browser'))
        self.assertFalse(self.call(1, 'stop').accepted)
        self.assertEqual(self.calls, [])
        result = self.policy.request('a', 2, 100, 'stop', '', self.dispatch,
                                     enabled=False, now_wall=100, now_mono=1)
        self.assertFalse(result.accepted)
        self.assertEqual(self.calls, [])

    def test_feedback_regression_and_lifecycle_require_acknowledgement(self):
        self.call(1, 'stop')
        self.policy.observe(state(state='Running'))
        self.assertFalse(self.call(2, 'start').accepted)
        self.policy.observe(state())
        self.call(3, 'stop')
        self.call(4, 'start')
        self.policy.observe(state(state='Running', can_feedback_frames=12,
                                  pose_sequence=5, sequence=5))
        self.policy.observe(state(state='Running', can_feedback_frames=7,
                                  pose_sequence=1, sequence=1))
        self.assertFalse(self.call(5, 'hold').accepted)
        self.assertTrue(self.policy.needs_stop)

    def test_stale_feedback_stop_is_forwarded_but_cannot_enable_stale_start(self):
        self.policy.observe(state(pose_age_ms=1000))
        self.assertTrue(self.call(1, 'stop').accepted)
        self.assertFalse(self.call(2, 'start').accepted)
        self.assertEqual(len(self.calls), 1)

    def test_expiry_does_not_generate_stop_or_hold(self):
        self.call(1, 'stop')
        self.call(2, 'start')
        self.policy.expire(now_mono=1.2)
        self.assertTrue(self.policy.needs_stop)
        self.assertIsNone(self.policy.active)
        self.assertEqual([c[0] for c in self.calls], ['stop', 'start'])

    def test_lost_http_response_is_not_retried_and_requires_acknowledgement(self):
        self.call(1, 'stop')
        delivered = []
        def delivered_then_lost(command, direction):
            delivered.append(command)
            raise TimeoutError('response lost after dispatch')
        self.assertFalse(self.call(2, 'start', dispatch=delivered_then_lost).accepted)
        self.assertEqual(delivered, ['start'])
        self.assertFalse(self.call(3, 'start').accepted)
        self.assertTrue(self.policy.needs_stop)

    def test_rejected_or_malformed_receipt_cannot_leave_jog_active(self):
        for receipt in ({'accepted': False}, {'accepted': True, 'input_sequence': True, 'session': 1},
                        {'accepted': True, 'input_sequence': 65536, 'session': 1},
                        {'accepted': True, 'input_sequence': 4, 'session': 0}):
            with self.subTest(receipt=receipt):
                self.setUp()
                self.call(1, 'stop')
                self.assertFalse(self.call(2, 'start', dispatch=lambda *_: receipt).accepted)
                self.assertTrue(self.policy.needs_stop)
                self.assertIsNone(self.policy.active)

    def test_local_transport_rejects_remote_or_ambiguous_urls(self):
        for url in ('https://127.0.0.1:8082', 'http://example.com:8082',
                    'http://127.0.0.1:8083', 'http://user@localhost:8082',
                    'http://localhost:8082/command', 'http://localhost:8082?source=browser'):
            with self.subTest(url=url), self.assertRaises(ValueError):
                SimulatorClient(url)
        self.assertEqual(SimulatorClient('http://localhost:8082/').base_url, 'http://localhost:8082')


if __name__ == '__main__':
    unittest.main()
