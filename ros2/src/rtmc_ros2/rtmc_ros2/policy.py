"""ROS-independent validation and volatile jog ownership policy.

The drive and its collision supervisor remain authoritative. This policy never
turns a failure into Stop: Stop is an explicit client acknowledgement only.
"""

import math
import time
from dataclasses import dataclass

DIRECTIONS = frozenset((
    'R-up', 'R-down', 'R-left', 'R-right',
    'L-up', 'L-down', 'L-left', 'L-right', 'A3-left', 'A3-right',
))
MAX_CLIENTS = 32


@dataclass(frozen=True)
class JogResult:
    accepted: bool
    message: str
    input_sequence: int = 0
    session: int = 0


def feedback_check(state, elapsed_ms=0.0):
    """Return (valid, reason, axis radians), using the runtime's CAN age."""
    if not isinstance(state, dict):
        return False, 'simulator disconnected', None
    try:
        if state.get('simulation_only') is not True:
            raise ValueError('simulation provenance missing')
        if state.get('pose_source') != 'drive_can_feedback':
            raise ValueError('coherent CAN pose source missing')
        if state.get('feedback_valid') is not True:
            raise ValueError('CAN feedback invalid')
        if state.get('protocol_version') != 4 or state.get('collision_enabled') is not True:
            raise ValueError('expected supervised revision-4 runtime')
        if isinstance(state.get('pose_age_ms'), bool) or not isinstance(state.get('pose_age_ms'), (int, float)):
            raise ValueError('invalid CAN feedback age')
        age = float(state['pose_age_ms']) + float(elapsed_ms)
        if not math.isfinite(age) or age < 0 or age > 250:
            raise ValueError('CAN feedback stale')
        frames = state['can_feedback_frames']
        pose_sequence = state['pose_sequence']
        if (type(frames) is not int or frames < 7 or type(pose_sequence) is not int or
                pose_sequence <= 0 or type(state.get('sequence')) is not int or
                state['sequence'] != pose_sequence):
            raise ValueError('coherent CAN frame set missing')
        axes = state['axles_deg']
        if not isinstance(axes, list) or len(axes) != 5:
            raise ValueError('expected five axes')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or abs(v) > 360 for v in axes):
            raise ValueError('invalid axis feedback')
        radians = [math.radians(v) for v in axes]
        return True, 'fresh coherent CAN feedback', radians
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return False, str(exc), None


class JogPolicy:
    def __init__(self, max_clients=MAX_CLIENTS):
        self.max_clients = max_clients
        self.sequences = {}
        self.active = None  # (client_id, direction, last_input_monotonic)
        self.needs_stop = True  # bridge restart never acknowledges a drive latch
        self.state = None
        self.feedback_fresh = False
        self.feedback_reason = 'no feedback observed'
        self.connected = False

    def observe(self, state, elapsed_ms=0.0):
        was_connected = self.connected
        previous = self.state if was_connected else None
        self.state = state if isinstance(state, dict) else None
        self.connected = isinstance(state, dict)
        self.feedback_fresh, self.feedback_reason, _ = feedback_check(state, elapsed_ms)
        if not self.connected or not self.feedback_fresh:
            self.needs_stop = True
            self.active = None
        if not was_connected and self.connected:
            self.needs_stop = True
        if previous and self.connected:
            try:
                if (state['pose_sequence'] < previous['pose_sequence'] or
                        state['can_feedback_frames'] < previous['can_feedback_frames']):
                    self.needs_stop = True
                    self.active = None
                    self.feedback_fresh = False
                    self.feedback_reason = 'simulator feedback counter regressed; explicit Stop required'
            except (KeyError, TypeError):
                self.needs_stop = True
                self.active = None
        if self.connected and state.get('state') in ('AvoidanceLatched', 'FaultLatched'):
            self.needs_stop = True
            self.active = None

    def expire(self, now_mono=None):
        now_mono = time.monotonic() if now_mono is None else now_mono
        if self.active and now_mono - self.active[2] > .150:
            self.active = None
            self.needs_stop = True

    def request(self, client_id, sequence, stamp_seconds, command, direction,
                dispatch, *, enabled, now_wall=None, now_mono=None):
        now_wall = time.time() if now_wall is None else now_wall
        now_mono = time.monotonic() if now_mono is None else now_mono
        self.expire(now_mono)
        if not isinstance(client_id, str) or not client_id or len(client_id) > 128:
            return JogResult(False, 'client_id must be 1 to 128 characters')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 < sequence <= 2**64 - 1:
            return JogResult(False, 'sequence must be a positive uint64')
        if client_id not in self.sequences and len(self.sequences) >= self.max_clients:
            return JogResult(False, 'client history full; restart bridge to clear it')
        if sequence <= self.sequences.get(client_id, 0):
            return JogResult(False, 'sequence must strictly increase per client')
        try:
            age = now_wall - float(stamp_seconds)
        except (ValueError, TypeError, OverflowError):
            return JogResult(False, 'invalid wall-clock timestamp')
        if not math.isfinite(age) or age > .150 or age < -.050:
            return JogResult(False, 'stale or future wall-clock timestamp')
        self.sequences[client_id] = sequence
        if command not in ('start', 'hold', 'stop'):
            return JogResult(False, 'command must be start, hold, or stop')
        if command != 'stop' and direction not in DIRECTIONS:
            return JogResult(False, 'unknown jog direction')
        if not enabled:
            return JogResult(False, 'commands disabled; set enable_commands true')
        if not self.connected:
            return JogResult(False, 'simulator disconnected')
        if self.state.get('command_source') != 'ros2':
            return JogResult(False, 'simulator command_source is not ros2')
        if command != 'stop':
            if not self.feedback_fresh:
                return JogResult(False, self.feedback_reason)
            if self.needs_stop:
                return JogResult(False, 'explicit Stop required before Start')
            required_states = ('Disarmed',) if command == 'start' else ('Preflight', 'Running')
            if self.state.get('state') not in required_states:
                self.needs_stop = True
                self.active = None
                return JogResult(False, 'controller lifecycle requires explicit Stop')
            if command == 'start' and self.active:
                return JogResult(False, 'another jog is active; Stop required')
            if command == 'hold' and (not self.active or
                                      self.active[:2] != (client_id, direction)):
                return JogResult(False, 'Hold requires the active client and direction')
        try:
            response = dispatch(command, direction)
            if response.get('accepted') is not True:
                raise ValueError('HTTP command rejected')
            input_sequence = response['input_sequence']
            session = response['session']
            if (type(input_sequence) is not int or not 0 < input_sequence <= 65535 or
                    type(session) is not int or not 0 <= session <= 2**32 - 1):
                raise ValueError('invalid HTTP command receipt')
        except Exception as exc:
            self.needs_stop = True
            self.active = None
            return JogResult(False, 'command outcome uncertain: ' + str(exc))
        if command == 'stop':
            self.active = None
            self.needs_stop = False
        else:
            if session == 0:
                self.active = None
                self.needs_stop = True
                return JogResult(False, 'command returned no controller session', input_sequence, session)
            self.active = (client_id, direction, now_mono)
        return JogResult(True, 'HTTP input dispatched; motion authorization remains with drive',
                         input_sequence, session)
