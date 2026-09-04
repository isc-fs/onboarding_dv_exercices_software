"""Toy ROS 2 talker — fill in the publisher.

Build/run (inside a ROS 2 sourced environment, e.g. the pipeline container).
See ``onboarding/GUIDE.md`` § ROS pub/sub.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("onboarding_talker")
        # === STUDENT TODO ===
        # 1. Create a publisher:
        #    - message type: String (already imported)
        #    - topic name: "onboarding/chatter"
        #    - queue size: 10
        #    - store the handle on self._pub
        # 2. Create a 1.0 second timer whose callback is self._tick
        # Look up create_publisher / create_timer in the rclpy docs or
        # another node in pipeline/ — do not copy from solutions yet.
        raise NotImplementedError("STUDENT TODO: publisher + timer")
        # === END TODO ===
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


if __name__ == "__main__":
    main()
