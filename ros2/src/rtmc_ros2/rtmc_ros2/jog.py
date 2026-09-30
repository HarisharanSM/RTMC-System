"""Bounded, explicit jog sequence example: Stop, Start, Hold, Stop."""

import argparse
import time
import uuid

import rclpy
from rtmc_interfaces.srv import Jog

from .policy import DIRECTIONS


def main():
    parser = argparse.ArgumentParser(description='Bounded RTMC simulation jog')
    parser.add_argument('direction', nargs='?', choices=sorted(DIRECTIONS))
    parser.add_argument('--stop', action='store_true', help='explicit Stop / acknowledge only')
    parser.add_argument('--duration', type=float, default=.5,
                        help='jog duration in seconds (0 < duration <= 2)')
    args = parser.parse_args()
    if (args.stop and args.direction is not None) or (not args.stop and args.direction is None):
        parser.error('choose a direction to jog, or --stop alone')
    if not 0 < args.duration <= 2:
        parser.error('duration must be between 0 and 2 seconds')
    rclpy.init()
    node = rclpy.create_node('rtmc_jog_example')
    client = node.create_client(Jog, '/rtmc/jog')
    client_id = 'example-' + str(uuid.uuid4())
    sequence = 0

    def send(command):
        nonlocal sequence
        sequence += 1
        request = Jog.Request()
        instant_ns = time.time_ns()
        request.stamp.sec = instant_ns // 1_000_000_000
        request.stamp.nanosec = instant_ns % 1_000_000_000
        request.client_id = client_id
        request.sequence = sequence
        request.command = command
        request.direction = args.direction or ''
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=.3)
        if not future.done() or future.result() is None:
            raise RuntimeError(command + ' service response timed out')
        result = future.result()
        if not result.accepted:
            raise RuntimeError(command + ' rejected: ' + result.message)
        print(command, result.message, 'session', result.session)

    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('/rtmc/jog unavailable')
        send('stop')  # explicit operator acknowledgement before the first Start
        if args.stop:
            return
        send('start')
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            time.sleep(min(.05, max(0, end - time.monotonic())))
            if time.monotonic() < end:
                send('hold')
        send('stop')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
