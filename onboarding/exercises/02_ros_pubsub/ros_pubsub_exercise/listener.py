"""Toy ROS 2 listener — fill in the subscription."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self) -> None:
        super().__init__("onboarding_listener")
        # === STUDENT TODO ===
        # Create a subscription that:
        #   - listens for String messages
        #   - on topic "onboarding/chatter"
        #   - calls self._on_msg when a message arrives
        #   - uses queue size 10
        # You do not need to store the subscription handle for this toy.
        raise NotImplementedError("STUDENT TODO: subscription")
        # === END TODO ===

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


if __name__ == "__main__":
    main()
