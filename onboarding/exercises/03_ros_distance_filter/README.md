# Exercise 03 — Distance filter node

Slightly harder than talker/listener: **one node that both subscribes and
publishes**, plus a **parameter**.

## Behaviour

| I/O | Type | Topic |
|-----|------|-------|
| In | `geometry_msgs/Point` | `onboarding/point_in` |
| Out (always) | `std_msgs/Float64` | `onboarding/distance` |
| Out (gated) | `geometry_msgs/Point` | `onboarding/point_out` |

On every incoming point `(x, y, z)`:

1. Compute planar distance `r = hypot(x, y)`.
2. Publish `r` on `onboarding/distance`.
3. If `r >= min_range` (ROS parameter, default `1.0`), also forward the
   original point on `onboarding/point_out`.

## Run (after filling TODOs)

```bash
# terminal 1
ros2 run ros_distance_filter filter

# terminal 2 — should see distance, and point_out only when far enough
ros2 topic echo /onboarding/distance
ros2 topic echo /onboarding/point_out

# terminal 3 — publish test points
ros2 topic pub --once /onboarding/point_in geometry_msgs/msg/Point "{x: 0.2, y: 0.0, z: 0.0}"
ros2 topic pub --once /onboarding/point_in geometry_msgs/msg/Point "{x: 3.0, y: 4.0, z: 0.0}"
```

Override the threshold:

```bash
ros2 run ros_distance_filter filter --ros-args -p min_range:=2.5
```
