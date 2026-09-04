"""Solution: toy talker."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("onboarding_talker")
        self._pub = self.create_publisher(String, "onboarding/chatter", 10)
        self.create_timer(1.0, self._tick)
        self._i = 0

    def _tick(self) -> None:
        msg = String()
        msg.data = f"hello {self._i}"
        self._pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self._i += 1


def main() -> None:
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
