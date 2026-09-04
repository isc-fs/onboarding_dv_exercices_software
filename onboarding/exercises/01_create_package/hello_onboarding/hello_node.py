"""Minimal hello node — complete the TODOs."""
from __future__ import annotations

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self) -> None:
        # === STUDENT TODO ===
        # Call super().__init__ with a unique node name, e.g. "hello_onboarding".
        # Create a 1.0 s timer that calls self._tick.
        raise NotImplementedError("STUDENT TODO: HelloNode.__init__")
        # === END TODO ===

    def _tick(self) -> None:
        # === STUDENT TODO ===
        # Log an info message with self.get_logger().info(...)
        raise NotImplementedError("STUDENT TODO: HelloNode._tick")
        # === END TODO ===


def main() -> None:
    rclpy.init()
    node = HelloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
