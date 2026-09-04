"""Distance filter node — subscribe + dual publish + parameter."""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64


class DistanceFilter(Node):
    def __init__(self) -> None:
        super().__init__("distance_filter")
        # === STUDENT TODO ===
        # 1. Declare a float parameter named "min_range" with default 1.0.
        #    Read it into self._min_range.
        # 2. Create a publisher for Float64 on "onboarding/distance".
        # 3. Create a publisher for Point on "onboarding/point_out".
        # 4. Create a subscription for Point on "onboarding/point_in"
        #    that calls self._on_point.
        raise NotImplementedError("STUDENT TODO: params + pubs + sub")
        # === END TODO ===

    def _on_point(self, msg: Point) -> None:
        # === STUDENT TODO ===
        # r = hypot(msg.x, msg.y)
        # Always publish r as Float64 on the distance topic.
        # If r >= self._min_range, also publish the original Point on point_out.
        raise NotImplementedError("STUDENT TODO: filter callback")
        # === END TODO ===


def main() -> None:
    rclpy.init()
    node = DistanceFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
