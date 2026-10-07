"""S15.1: two processes exchanging integer sequence numbers through a ROS2 topic."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32


class TopicNode(Node):
    def __init__(self, role, period, count):
        super().__init__("s15_1_" + role)
        self.count = count
        self.sent = 0
        self.received = []
        # Both endpoints request reliable delivery; volatile does not replay old data.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        if role == "publisher":
            # Returns a publisher handle; Int32 is the wire type, not a Python int.
            self.publisher = self.create_publisher(Int32, "/s15_1/sequence", qos)
            # Period is seconds; the executor invokes this callback during spinning.
            self.timer = self.create_timer(period, self.publish_next)
        else:
            # Keep the subscription handle; callbacks receive an Int32 message.
            self.subscription = self.create_subscription(
                Int32, "/s15_1/sequence", self.receive, qos)

    def publish_next(self):
        # Wait for discovery before the finite sequence; timeout still bounds waiting.
        if self.sent >= self.count or self.publisher.get_subscription_count() == 0:
            return
        msg = Int32()
        msg.data = self.sent
        self.publisher.publish(msg)  # Enqueues a message; does not await reception.
        self.get_logger().info(f"sent={msg.data}")
        self.sent += 1

    def receive(self, msg):
        self.received.append(msg.data)
        self.get_logger().info(f"received={msg.data}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("publisher", "subscriber"))
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=20.0)
    args, ros_args = parser.parse_known_args()
    if (not math.isfinite(args.period) or args.period <= 0
            or not math.isfinite(args.timeout) or args.timeout <= 0
            or not 1 <= args.count <= 100000):
        parser.error("period/timeout must be finite and positive; count must be 1..100000")
    rclpy.init(args=ros_args)  # Initializes ROS context; no node returned.
    node = None
    try:
        node = TopicNode(args.role, args.period, args.count)
        deadline = time.monotonic() + args.timeout  # Wall-time budget, not simulation time.
        while rclpy.ok() and time.monotonic() < deadline:
            # Runs ready timer/subscription callbacks in place, without returning state.
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.role == "subscriber" and len(node.received) >= args.count:
                passed = node.received == list(range(args.count))
                print(f"sequence_passed={passed}; received={node.received}")
                return 0 if passed else 1
        # Publisher remains alive to allow queued traffic to drain; its count is not an ACK.
        if args.role == "publisher":
            print(f"published={node.sent}/{args.count}; reception_requires_subscriber_check")
            return 0 if node.sent == args.count else 1
        print(f"timeout; received={node.received}")
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
