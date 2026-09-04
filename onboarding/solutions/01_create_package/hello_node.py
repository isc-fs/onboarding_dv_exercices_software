"""Solution: hello node."""
from __future__ import annotations

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self) -> None:
        super().__init__("hello_onboarding")
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self.get_logger().info("Hello from hello_onboarding")


def main() -> None:
    rclpy.init()
    node = HelloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
