"""ROS-independent readiness checks used by the native smoke programs."""

import math
import time


def diagnostic_details(message):
    if message is None or not getattr(message, 'status', None):
        return 'no diagnostic status received'
    status = message.status[0]
    values = {item.key: item.value for item in getattr(status, 'values', ())}
    stamp_ns = joint_stamp_ns(message)
    age_ms = (time.time_ns() - stamp_ns) / 1e6 if stamp_ns is not None else None
    return 'level=%r message=%r age_ms=%r values=%r' % (
        status.level, status.message, age_ms, values)


def healthy_diagnostic(message, ok_level, *, now_ns=None, max_age_ns=250_000_000,
                       allowed_lifecycle=('Disarmed',)):
    if message is None or not getattr(message, 'status', None):
        return False
    now_ns = time.time_ns() if now_ns is None else now_ns
    stamp_ns = joint_stamp_ns(message)
    if stamp_ns is None or not -50_000_000 <= now_ns - stamp_ns <= max_age_ns:
        return False
    status = message.status[0]
    values = {item.key: item.value for item in getattr(status, 'values', ())}
    return (status.level == ok_level and values.get('connection') == 'connected' and
            values.get('feedback_fresh') == 'true' and
            values.get('lifecycle') in allowed_lifecycle)


def joint_stamp_ns(sample):
    stamp = sample.header.stamp
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int or
            stamp.sec <= 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def select_transform_sample(samples, can_transform, *, now_ns=None,
                            max_age_ns=250_000_000, scan_limit=20, min_stamp_ns=0):
    """Pick recent CAN joint feedback with TF available at its exact stamp."""
    now_ns = time.time_ns() if now_ns is None else now_ns
    for sample in reversed(samples[-scan_limit:]):
        stamp_ns = joint_stamp_ns(sample)
        if stamp_ns is None or stamp_ns < min_stamp_ns or not -50_000_000 <= now_ns - stamp_ns <= max_age_ns:
            continue
        if (list(getattr(sample, 'name', ())) != ['A1', 'A2', 'A3', 'A4', 'A5'] or
                len(getattr(sample, 'position', ())) != 5 or
                not all(math.isfinite(value) for value in sample.position)):
            continue
        if can_transform(sample):
            return sample
    return None
