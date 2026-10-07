"""S15.3: FollowJointTrajectory lifecycle with one software joint (no physics)."""

import argparse
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION = "/s15_3/follow_joint_trajectory"
JOINT = "teaching_joint"
DT = 0.02  # cooperative checks every ~20ms, not a hard real-time deadline


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def validate(goal):
    """Only this lesson's two-point, position-only, immediate single-joint subset."""
    t = goal.trajectory
    if t.joint_names != [JOINT]:
        return "invalid_joint_names"
    if (t.header.stamp.sec or t.header.stamp.nanosec or t.header.frame_id
            or goal.multi_dof_trajectory.joint_names or goal.multi_dof_trajectory.points
            or goal.path_tolerance or goal.goal_tolerance
            or goal.component_path_tolerance or goal.component_goal_tolerance
            or seconds(goal.goal_time_tolerance)):
        return "unsupported_fields"
    if len(t.points) != 2:
        return "requires_two_points"
    for p in t.points:
        if (len(p.positions) != 1 or not math.isfinite(p.positions[0])
                or abs(p.positions[0]) > 1 or p.velocities or p.accelerations or p.effort):
            return "invalid_position_only_point"
        if p.time_from_start.sec < 0 or not 0 <= p.time_from_start.nanosec < 1000000000:
            return "invalid_duration"
    if seconds(t.points[0].time_from_start) != 0:
        return "first_time_must_be_zero"
    if not 0.2 <= seconds(t.points[1].time_from_start) <= 5:
        return "duration_must_be_0.2_to_5_seconds"
    return None


class TrajectoryServer(Node):
    def __init__(self, fail_after):
        super().__init__("s15_3_server")
        self.lock = threading.Lock()
        self.busy = False
        self.q = 0.0  # rad, software reference state; not a measured physical joint
        self.fail_after = fail_after
        # Reentrant callbacks + two executor workers let cancel run during execution.
        self.action = ActionServer(
            self, FollowJointTrajectory, ACTION, execute_callback=self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())

    def accept(self, goal):
        reason = validate(goal)
        with self.lock:
            if not reason and self.busy:
                reason = "busy"
            if not reason and abs(goal.trajectory.points[0].positions[0] - self.q) > 1e-6:
                reason = "start_does_not_match_current_reference"
            if reason:
                self.get_logger().info(f"goal_rejected: {reason}")
                return GoalResponse.REJECT
            self.busy = True  # reserve before execute starts; one active goal only
        self.get_logger().info("goal_accepted")
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        self.get_logger().info("cancel_request_accepted")
        return CancelResponse.ACCEPT  # approval is not the terminal CANCELED state

    def execute(self, handle):
        result = FollowJointTrajectory.Result()
        start = time.monotonic()
        p0, p1 = handle.request.trajectory.points
        duration = seconds(p1.time_from_start)
        try:
            while True:
                # Stop before any next reference write when cancellation is observed.
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.error_string = "canceled: reference held; no physical stop guarantee"
                    self.get_logger().info(f"terminal=CANCELED; held_q={self.q:.6f}")
                    return result
                elapsed = time.monotonic() - start
                if self.fail_after >= 0 and elapsed >= self.fail_after:
                    handle.abort()
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "injected execution fault; not measured tracking error"
                    self.get_logger().info(f"terminal=ABORTED; held_q={self.q:.6f}")
                    return result
                u = min(elapsed / duration, 1.0)
                with self.lock:
                    self.q = p0.positions[0] + u * (p1.positions[0] - p0.positions[0])
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = [JOINT]
                feedback.desired.positions = [self.q]
                feedback.actual.positions = [self.q]  # ideal teaching echo, not measurement
                feedback.error.positions = [0.0]
                handle.publish_feedback(feedback)
                self.get_logger().info(f"reference_update: q={self.q:.6f}")
                if u == 1.0:
                    handle.succeed()
                    result.error_code = result.SUCCESSFUL
                    result.error_string = "reference_completed; no dynamics validation"
                    self.get_logger().info(f"terminal=SUCCEEDED; q={self.q:.6f}")
                    return result
                time.sleep(DT)
        except Exception as error:
            if handle.is_active:
                handle.abort()
            result.error_code = result.INVALID_GOAL
            result.error_string = f"execution_exception: {type(error).__name__}"
            return result
        finally:
            with self.lock:
                self.busy = False


def wait_future(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.done()


def run_client(node, args):
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=3):
        print("status=server_unavailable")
        return 3
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [JOINT]
    for q, t in ((args.start, 0.0), (args.target, args.duration)):
        point = JointTrajectoryPoint()
        point.positions = [q]
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = round((t - int(t)) * 1e9)
        if point.time_from_start.nanosec == 1000000000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec = 0
        goal.trajectory.points.append(point)
    feedback_count = 0

    def feedback(message):
        nonlocal feedback_count
        feedback_count += 1
        print(f"feedback={feedback_count}; q={message.feedback.actual.positions[0]:.6f}", flush=True)

    pending = client.send_goal_async(goal, feedback_callback=feedback)
    if not wait_future(node, pending, 3):
        print("status=goal_response_timeout; remote_state_unknown")
        return 4
    handle = pending.result()
    if not handle.accepted:
        print("status=goal_rejected")
        return 2
    print("status=goal_accepted", flush=True)
    result_future = handle.get_result_async()
    start = time.monotonic()
    cancel_future = None
    timed_out = False
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=DT)
        elapsed = time.monotonic() - start
        timed_out = timed_out or elapsed >= args.result_timeout
        if cancel_future is None and (timed_out or (args.cancel_after >= 0 and elapsed >= args.cancel_after)):
            cancel_future = handle.cancel_goal_async()
            print("status=cancel_requested", flush=True)
        if timed_out and elapsed >= args.result_timeout + 2:
            print("status=terminal_timeout; remote_state_unknown")
            return 4
    if cancel_future is not None:
        acknowledged = wait_future(node, cancel_future, 1)
        accepted = acknowledged and bool(cancel_future.result().goals_canceling)
        print(f"cancel_acknowledged={accepted}")
    wrapped = result_future.result()
    status = wrapped.status
    names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
             GoalStatus.STATUS_ABORTED: "ABORTED"}
    print(f"terminal={names.get(status, status)}; error_code={wrapped.result.error_code}; "
          f"message={wrapped.result.error_string}; feedback_count={feedback_count}")
    # Cancellation result has no dedicated control_msgs error code: use action status.
    if timed_out:
        return 4
    if args.cancel_after >= 0:
        return 0 if status == GoalStatus.STATUS_CANCELED else 1
    return 0 if status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("server", "client"))
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--target", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--cancel-after", type=float, default=-1.0)
    parser.add_argument("--fail-after", type=float, default=-1.0)
    parser.add_argument("--result-timeout", type=float, default=6.0)
    args, ros_args = parser.parse_known_args()
    if not all(math.isfinite(v) for v in (args.start,args.target,args.duration,args.cancel_after,args.fail_after,args.result_timeout)):
        parser.error("all values must be finite")
    if not 0 < args.duration <= 10 or args.result_timeout <= 0:
        parser.error("duration must be 0..10 seconds; result timeout positive")
    if any(v < 0 and v != -1 for v in (args.cancel_after,args.fail_after)):
        parser.error("cancel/fail after must be -1 (disabled) or nonnegative seconds")
    rclpy.init(args=ros_args)
    node = None
    executor = None
    try:
        if args.role == "server":
            node = TrajectoryServer(args.fail_after)
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            node.get_logger().info(f"action_ready: {ACTION}")
            executor.spin()
            return 0
        node = Node("s15_3_client")
        return run_client(node,args)
    except KeyboardInterrupt:
        return 130
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            if isinstance(node,TrajectoryServer):
                node.action.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
