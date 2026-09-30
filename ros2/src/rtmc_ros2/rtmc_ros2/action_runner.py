"""ROS-free, single-executor bounded jog action state machine."""

import math
import time
import uuid
from dataclasses import dataclass

from .policy import DIRECTIONS

MIN_DURATION = .1
MAX_DURATION = 5.0
HOLD_PERIOD = .05


@dataclass(frozen=True)
class ActionOutcome:
    completed: bool
    motion_observed: bool
    explicit_stop_required: bool
    message: str
    cancelled: bool = False


class ActionJogRunner:
    """Tick from the bridge's steady timer; never sleeps, retries or catches up."""

    def __init__(self, policy, direction, duration, dispatch, *, enabled, now_mono=None):
        if direction not in DIRECTIONS or isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or not MIN_DURATION <= duration <= MAX_DURATION:
            raise ValueError('direction must be known and duration_sec must be 0.1..5.0')
        self.policy = policy
        self.owner = object()
        accepted, reason = policy.reserve_action(self.owner, direction, enabled=enabled)
        if not accepted:
            raise ValueError(reason)
        self.direction = direction
        self.duration = float(duration)
        self.dispatch = dispatch
        self.client_id = 'action-' + str(uuid.uuid4())
        self.sequence = 0
        self.started = None
        self.last_input = None
        self.next_hold = None
        self.motion_observed = False
        self.baseline_axes = tuple(policy.state['axles_deg'])
        self.baseline_sequence = policy.state['pose_sequence']
        self.pending_stop = None
        self.outcome = None

    def _send(self, command, now_mono, now_wall):
        self.sequence += 1
        return self.policy.request(self.client_id, self.sequence, now_wall,
                                   command, self.direction, self.dispatch,
                                   enabled=True, now_wall=now_wall,
                                   now_mono=now_mono, action_owner=self.owner)

    def _finish(self, completed, message, *, cancelled=False, stop_required=None):
        required = self.policy.needs_stop if stop_required is None else stop_required
        if required:
            self.policy.needs_stop = True
            self.policy.active = None
        self.policy.release_action(self.owner)
        self.outcome = ActionOutcome(completed, self.motion_observed,
            required,
            message, cancelled)
        return self.outcome

    def _unhealthy_reason(self, now_mono):
        if not self.policy.connected:
            return 'simulator disconnected'
        if not self.policy.feedback_fresh:
            return 'feedback invalid: ' + self.policy.feedback_reason
        if self.policy.needs_stop:
            return 'policy requires explicit Stop'
        current = self.policy.state or {}
        if current.get('command_source') != 'ros2':
            return 'simulator command source changed'
        if current.get('state') not in ('Preflight', 'Running'):
            return 'controller lifecycle is ' + str(current.get('state'))
        if self.policy.active is None or self.policy.active[:2] != (self.client_id, self.direction):
            return 'action no longer owns active input'
        if now_mono - self.policy.active[2] > .150:
            return 'input renewal expired'
        return None

    def step(self, *, now_mono=None, now_wall=None, cancel=False):
        if self.outcome is not None:
            return self.outcome
        now_mono = time.monotonic() if now_mono is None else now_mono
        now_wall = time.time() if now_wall is None else now_wall
        self.policy.expire(now_mono)
        if self.policy.action_owner is not self.owner:
            return self._finish(False, 'action interrupted by explicit service Stop or another owner',
                                stop_required=self.policy.needs_stop)
        if self.pending_stop is not None:
            was_cancelled, stopped_at = self.pending_stop
            current = self.policy.state or {}
            if (self.policy.connected and self.policy.feedback_fresh and
                    current.get('command_source') == 'ros2' and
                    current.get('state') == 'Disarmed' and not self.policy.needs_stop):
                return self._finish(not was_cancelled,
                    ('cancelled; fresh Disarmed feedback confirmed' if was_cancelled else
                     'bounded input completed; fresh Disarmed feedback confirmed'),
                    cancelled=was_cancelled, stop_required=False)
            if self.policy.needs_stop or not self.policy.feedback_fresh or now_mono - stopped_at >= .150:
                return self._finish(False, 'Stop dispatched but Disarmed feedback unconfirmed; explicit Stop required',
                                    stop_required=True)
            return None
        if self.started is None:
            if cancel:
                return self._finish(False, 'cancelled before Start', cancelled=True)
            if self.policy.needs_stop or not self.policy.feedback_fresh or self.policy.state.get('state') != 'Disarmed':
                return self._finish(False, 'startup state changed; explicit Stop required', stop_required=True)
            result = self._send('start', now_mono, now_wall)
            if not result.accepted:
                return self._finish(False, 'Start failed: ' + result.message, stop_required=True)
            self.started = now_mono
            self.last_input = now_mono
            self.next_hold = now_mono + HOLD_PERIOD
            return None
        current = self.policy.state or {}
        self.motion_observed |= (self.policy.feedback_fresh and
            type(current.get('pose_sequence')) is int and
            current['pose_sequence'] > self.baseline_sequence and
            isinstance(current.get('axles_deg'), list) and
            len(current['axles_deg']) == 5 and
            any(abs(a - b) > 1e-5 for a, b in zip(current['axles_deg'], self.baseline_axes)))
        unhealthy_reason = self._unhealthy_reason(now_mono)
        if unhealthy_reason is not None:
            return self._finish(False, unhealthy_reason + '; explicit Stop required',
                                stop_required=True)
        if cancel or now_mono - self.started >= self.duration:
            result = self._send('stop', now_mono, now_wall)
            if not result.accepted:
                return self._finish(False, 'Stop outcome uncertain: ' + result.message,
                                    stop_required=True)
            self.pending_stop = (cancel, now_mono)
            return None
        if now_mono >= self.next_hold:
            # A delayed timer emits only one Hold. A gap beyond the permit
            # window aborts above instead of sending a burst or late renewal.
            result = self._send('hold', now_mono, now_wall)
            if not result.accepted:
                return self._finish(False, 'Hold failed: ' + result.message, stop_required=True)
            self.last_input = now_mono
            self.next_hold = now_mono + HOLD_PERIOD
        return None
