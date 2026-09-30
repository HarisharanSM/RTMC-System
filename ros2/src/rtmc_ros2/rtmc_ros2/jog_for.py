"""Action client for a bounded jog after explicit operator Stop."""

import argparse
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rtmc_interfaces.action import JogFor

from .action_runner import MAX_DURATION, MIN_DURATION
from .policy import DIRECTIONS


def main():
    parser = argparse.ArgumentParser(description='Bounded RTMC simulation jog action')
    parser.add_argument('direction', choices=sorted(DIRECTIONS))
    parser.add_argument('--duration', type=float, default=.5, help='duration in seconds, 0.1..5.0')
    parser.add_argument('--cancel-after', type=float, help='request cancellation after this many seconds')
    args = parser.parse_args()
    if not MIN_DURATION <= args.duration <= MAX_DURATION:
        parser.error('duration must be 0.1..5.0 seconds')
    if args.cancel_after is not None and not 0 <= args.cancel_after < args.duration:
        parser.error('--cancel-after must be nonnegative and less than duration')
    rclpy.init()
    node = rclpy.create_node('rtmc_jog_for_client')
    client = ActionClient(node, JogFor, '/rtmc/jog_for')
    try:
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('/rtmc/jog_for unavailable')
        goal = JogFor.Goal()
        goal.direction = args.direction
        goal.duration_sec = args.duration
        sent = time.monotonic()
        future = client.send_goal_async(goal, feedback_callback=lambda msg:
            print('elapsed %.2fs %s feedback_valid=%s' %
                  (msg.feedback.elapsed_sec, msg.feedback.controller_state,
                   msg.feedback.feedback_valid), flush=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or future.result() is None:
            raise RuntimeError('goal response timed out')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('goal rejected: prior explicit /rtmc/jog Stop and fresh Disarmed feedback required')
        result_future = handle.get_result_async()
        cancelled = False
        cancel_future = None
        deadline = time.monotonic() + args.duration + 3.0
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=.02)
            if args.cancel_after is not None and not cancelled and time.monotonic() - sent >= args.cancel_after:
                cancel_future = handle.cancel_goal_async()
                cancelled = True
            if time.monotonic() >= deadline:
                raise RuntimeError('action result timed out; outcome uncertain, explicit Stop required')
        if cancel_future is not None and cancel_future.done():
            response = cancel_future.result()
            if response is None or not response.goals_canceling:
                raise RuntimeError('cancellation was rejected or uncertain')
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError('action result unavailable')
        result = wrapped.result
        print('%s; completed=%s motion_observed=%s explicit_stop_required=%s' %
              (result.message, result.completed, result.motion_observed,
               result.explicit_stop_required), flush=True)
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if not cancelled or result.completed or result.explicit_stop_required:
                raise RuntimeError('cancellation ended without a confirmed safe Stop: ' + result.message)
        elif wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.completed or result.explicit_stop_required:
            raise RuntimeError(result.message)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
