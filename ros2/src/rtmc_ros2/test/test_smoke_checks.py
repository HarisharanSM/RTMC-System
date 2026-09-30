import unittest
from types import SimpleNamespace as NS

from rtmc_ros2.smoke_checks import (
    diagnostic_details, healthy_diagnostic, select_transform_sample,
)


NOW = 1_000_000_000_000


def diagnostic(level=b'\x00', *, connection='connected', fresh='true',
               lifecycle='Disarmed', message='fresh coherent CAN feedback',
               stamp_ns=NOW - 50_000_000):
    values = [NS(key='connection', value=connection),
              NS(key='feedback_fresh', value=fresh),
              NS(key='lifecycle', value=lifecycle)]
    return NS(header=NS(stamp=NS(sec=stamp_ns // 1_000_000_000,
                                nanosec=stamp_ns % 1_000_000_000)),
              status=[NS(level=level, message=message, values=values)])


def joint(stamp_ns, positions=(0., 0., 0., 0., 0.)):
    return NS(header=NS(stamp=NS(sec=stamp_ns // 1_000_000_000,
                                nanosec=stamp_ns % 1_000_000_000)),
              name=['A1', 'A2', 'A3', 'A4', 'A5'], position=positions)


class SmokeChecksTest(unittest.TestCase):
    def test_octet_healthy_readiness_and_unhealthy_details(self):
        ok = b'\x00'
        self.assertTrue(healthy_diagnostic(diagnostic(), ok, now_ns=NOW))
        self.assertFalse(healthy_diagnostic(diagnostic(), 0, now_ns=NOW))
        for bad in (diagnostic(level=b'\x02', message='stale CAN feedback'),
                    diagnostic(fresh='false'), diagnostic(connection='disconnected'),
                    diagnostic(lifecycle='AvoidanceLatched'),
                    diagnostic(lifecycle='unknown'),
                    diagnostic(stamp_ns=NOW - 300_000_000)):
            self.assertFalse(healthy_diagnostic(bad, ok, now_ns=NOW))
        self.assertIn('stale CAN feedback', diagnostic_details(
            diagnostic(level=b'\x02', message='stale CAN feedback')))
        self.assertFalse(healthy_diagnostic(None, ok, now_ns=NOW))

    def test_new_sample_recovers_from_pre_listener_startup_sample(self):
        now = NOW
        early = joint(now - 200_000_000)
        later = joint(now - 50_000_000)
        available = lambda sample: sample is later
        self.assertIsNone(select_transform_sample([early], available, now_ns=now))
        self.assertIs(select_transform_sample([early, later], available, now_ns=now), later)

    def test_sample_must_follow_check_start_even_if_old_tf_is_available(self):
        old = joint(NOW - 100_000_000)
        new = joint(NOW - 20_000_000)
        self.assertIsNone(select_transform_sample([old], lambda _: True,
            now_ns=NOW, min_stamp_ns=NOW - 50_000_000))
        self.assertIs(select_transform_sample([old, new], lambda _: True,
            now_ns=NOW, min_stamp_ns=NOW - 50_000_000), new)

    def test_stale_invalid_or_missing_tf_never_passes(self):
        now = NOW
        stale = joint(now - 2_000_000_000)
        invalid = joint(now - 50_000_000, positions=(0., float('nan'), 0., 0., 0.))
        current = joint(now - 10_000_000)
        self.assertIsNone(select_transform_sample([stale, invalid], lambda _: True,
                                                  now_ns=now))
        self.assertIsNone(select_transform_sample([current], lambda _: False,
                                                  now_ns=now))
        self.assertIs(select_transform_sample([current], lambda _: True,
                                              now_ns=now), current)


if __name__ == '__main__':
    unittest.main()
