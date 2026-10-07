"""S15.2: short cached-position validation via Trigger; not IK or execution."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVICE = "/s15_2/check_cached_pose"


def check_position(position, available):
    """Teaching gate: world-frame XYZ [m], not a robot reachability test."""
    if not available:
        return False, "pose_unavailable"
    if not all(math.isfinite(value) for value in position):
        return False, "invalid_position: nonfinite world XYZ [m]"
    x, y, z = position
    if not (-0.6 <= x <= 0.6 and -0.6 <= y <= 0.6 and 0.02 <= z <= 0.6):
        return False, "outside_teaching_workspace: world XYZ [m]"
    return True, "position_gate_passed: world XYZ [m]; no IK/collision/execution guarantee"


class PoseService(Node):
    def __init__(self, position, available, delay):
        super().__init__("s15_2_server")
        self.position = position
        self.available = available
        self.delay = delay
        # Returns a service handle. Callback receives request and mutable response.
        self.service = self.create_service(Trigger, SERVICE, self.respond)

    def respond(self, request, response):
        self.get_logger().info("request_received")
        # Failure injection only: blocks the single-thread executor; default is zero.
        if self.delay:
            time.sleep(self.delay)
        response.success, response.message = check_position(self.position, self.available)
        self.get_logger().info(f"response_ready: success={response.success}; {response.message}")
        return response  # Returning the populated object lets ROS2 send the response.


def call_once(node, wait_timeout, response_timeout):
    client = node.create_client(Trigger, SERVICE)  # Returns a client handle.
    if not client.wait_for_service(timeout_sec=wait_timeout):
        print("status=service_unavailable")
        return 3
    # Empty request triggers checking the server's cached position, not a new pose.
    future = client.call_async(Trigger.Request())  # Returns a Future, not a response.
    rclpy.spin_until_future_complete(node, future, timeout_sec=response_timeout)
    if not future.done():
        # Remove local bookkeeping; this does NOT cancel remote work.
        client.remove_pending_request(future)
        print("status=response_timeout; remote_work_not_cancelled")
        return 4
    try:
        response = future.result()
    except Exception as error:
        print(f"status=transport_error; {type(error).__name__}: {error}")
        return 5
    if response is None:
        print("status=transport_error; empty response")
        return 5
    print(f"status=response_received; success={response.success}; message={response.message}")
    return 0 if response.success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("server", "client"))
    parser.add_argument("--position", nargs=3, type=float, default=[0.3, 0.0, 0.1], metavar=("X", "Y", "Z"))
    parser.add_argument("--unavailable", action="store_true")
    parser.add_argument("--delay", type=float, default=0.0)
    parser.add_argument("--wait-timeout", type=float, default=3.0)
    parser.add_argument("--response-timeout", type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.delay) or not 0 <= args.delay <= 10:
        parser.error("delay must be finite and 0..10 seconds")
    if any(not math.isfinite(v) or v <= 0 for v in (args.wait_timeout, args.response_timeout)):
        parser.error("timeouts must be finite and positive seconds")
    # Nonfinite position intentionally reaches the server's explicit failure response.
    rclpy.init(args=ros_args)
    node = None
    try:
        if args.role == "server":
            node = PoseService(args.position, not args.unavailable, args.delay)
            node.get_logger().info(f"service_ready: {SERVICE}")
            rclpy.spin(node)
            return 0
        node = Node("s15_2_client")
        return call_once(node, args.wait_timeout, args.response_timeout)
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
