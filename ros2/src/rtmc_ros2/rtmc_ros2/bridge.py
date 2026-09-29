"""Single-threaded, loopback-only ROS 2 facade over pcan_demo."""

import fcntl
import os
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rtmc_interfaces.srv import Jog

from .policy import JogPolicy, feedback_check
from .transport import SimulatorClient


class Bridge(Node):
    def __init__(self):
        super().__init__('rtmc_bridge')
        self.declare_parameter('base_url', 'http://127.0.0.1:8082', ParameterDescriptor(read_only=True))
        self.declare_parameter('enable_commands', False, ParameterDescriptor(read_only=True))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RTMC bridge requires wall-clock time; use_sim_time must be false')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.enabled = self.get_parameter('enable_commands').value is True
        self.client = SimulatorClient(self.get_parameter('base_url').value)
        self.lock = open(os.path.join(tempfile.gettempdir(), 'rtmc_ros2_bridge.lock'), 'a+')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.lock.close()
            raise RuntimeError('another RTMC ROS bridge holds the process lock') from exc
        self.lock.seek(0)
        self.lock.truncate()
        self.lock.write(str(os.getpid()))
        self.lock.flush()
        self.policy = JogPolicy()
        self.observed_mono = 0.0
        self.state_pub = self.create_publisher(String, '/rtmc/state', 10)
        self.joints_pub = self.create_publisher(JointState, '/rtmc/joint_states', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/rtmc/diagnostics', 10)
        self.jog_service = self.create_service(Jog, '/rtmc/jog', self.on_jog)
        self.poll_timer = self.create_timer(.05, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def validate_parameters(self, parameters):
        if any(p.name == 'use_sim_time' and p.value for p in parameters):
            return SetParametersResult(successful=False, reason='bridge requires wall-clock time')
        return SetParametersResult(successful=True)

    def refresh(self):
        started = time.monotonic()
        try:
            state, raw = self.client.get_state()
            elapsed_ms = (time.monotonic() - started) * 1000
            self.policy.observe(state, elapsed_ms)
            # Keep request-start time so age includes transport latency on every
            # later check, rather than resetting freshness after a slow reply.
            self.observed_mono = started
            return raw
        except (OSError, ValueError) as exc:
            self.policy.observe(None)
            self.policy.feedback_reason = 'simulator disconnected: ' + str(exc)
            self.observed_mono = 0.0
            return None

    def poll(self):
        self.policy.expire()
        raw = self.refresh()
        if raw is not None:
            self.state_pub.publish(String(data=raw))
        self.publish_diagnostics()
        if self.policy.feedback_fresh:
            valid, _, axes = feedback_check(self.policy.state,
                (time.monotonic() - self.observed_mono) * 1000)
            if valid:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['A1', 'A2', 'A3', 'A4', 'A5']
                msg.position = axes
                self.joints_pub.publish(msg)

    def publish_diagnostics(self):
        current = self.policy.state or {}
        age = (time.monotonic() - self.observed_mono) * 1000 if self.observed_mono else float('inf')
        fresh, reason, _ = feedback_check(current, age)
        if not fresh:
            self.policy.feedback_fresh = False
            self.policy.feedback_reason = reason
            self.policy.needs_stop = True
            self.policy.active = None
        diag = DiagnosticStatus()
        diag.name = 'rtmc_ros2/bridge'
        diag.hardware_id = 'RTMC simulation'
        latched = current.get('state') in ('AvoidanceLatched', 'FaultLatched')
        diag.level = DiagnosticStatus.ERROR if not fresh or latched else DiagnosticStatus.OK
        diag.message = reason if fresh else self.policy.feedback_reason
        if latched:
            diag.message = str(current.get('reason', 'protective stop'))
        diag.values = [
            KeyValue(key='connection', value='connected' if self.policy.connected else 'disconnected'),
            KeyValue(key='feedback_fresh', value=str(fresh).lower()),
            KeyValue(key='lifecycle', value=str(current.get('state', 'unknown'))),
            KeyValue(key='protective_stop_reason', value=str(current.get('reason', ''))),
            KeyValue(key='command_source', value=str(current.get('command_source', 'unknown'))),
            KeyValue(key='explicit_stop_required', value=str(self.policy.needs_stop).lower()),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [diag]
        self.diagnostics_pub.publish(msg)

    def on_jog(self, request, response):
        # Refresh lifecycle even for back-to-back Stop/Start service calls.
        # Timestamp acceptance runs after this read and counts its latency.
        self.refresh()
        result = self.policy.request(
            request.client_id, request.sequence,
            request.stamp.sec + request.stamp.nanosec * 1e-9,
            request.command, request.direction, self.client.dispatch,
            enabled=self.enabled and not self.get_parameter('use_sim_time').value)
        response.accepted = result.accepted
        response.message = result.message
        response.input_sequence = result.input_sequence
        response.session = result.session
        return response

    def destroy_node(self):
        fcntl.flock(self.lock, fcntl.LOCK_UN)
        self.lock.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = Bridge()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
