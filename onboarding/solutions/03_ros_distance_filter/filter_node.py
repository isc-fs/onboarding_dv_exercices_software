"""Solution: distance filter node."""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64


class DistanceFilter(Node):
    def __init__(self) -> None:
        super().__init__("distance_filter")
        self.declare_parameter("min_range", 1.0)
        self._min_range = float(self.get_parameter("min_range").value)
        self._pub_dist = self.create_publisher(Float64, "onboarding/distance", 10)
        self._pub_point = self.create_publisher(Point, "onboarding/point_out", 10)
        self.create_subscription(Point, "onboarding/point_in", self._on_point, 10)

    def _on_point(self, msg: Point) -> None:
        r = math.hypot(msg.x, msg.y)
        dist = Float64()
        dist.data = r
        self._pub_dist.publish(dist)
        if r >= self._min_range:
            self._pub_point.publish(msg)


def main() -> None:
    rclpy.init()
    node = DistanceFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
