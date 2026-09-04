"""Solution: toy listener."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self) -> None:
        super().__init__("onboarding_listener")
        self.create_subscription(String, "onboarding/chatter", self._on_msg, 10)

    def _on_msg(self, msg: String) -> None:
        self.get_logger().info(f"I heard: {msg.data}")


def main() -> None:
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
