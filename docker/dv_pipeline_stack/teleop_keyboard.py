#!/usr/bin/env python3
"""
Keyboard teleop for IFSSIM — publishes fs_msgs/ControlCommand to /control_command.

  W / S  : throttle / brake
  A / D  : steer left / right
  SPACE  : full brake (emergency stop)
  Q      : quit

Run inside the container:
  docker exec -it ifssim-dv_pipeline_stack-1 python3 /dv_pipeline_stack_ws/teleop_keyboard.py
  docker exec -it ifssim-dv_pipeline_stack-1 python3 /dv_pipeline_stack_ws/teleop_keyboard.py \
      --max-throttle 0.3 --max-steer 0.4
"""

import argparse
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand

THROTTLE_STEP = 0.1
STEER_STEP    = 0.1
DECAY         = 0.85   # steering snaps back when key released

HELP = """
╔══════════════════════════════╗
║   IFSSIM Keyboard Teleop     ║
╠══════════════════════════════╣
║  W        throttle +         ║
║  S        brake +            ║
║  A / D    steer left / right ║
║  SPACE    emergency stop     ║
║  Q        quit               ║
╚══════════════════════════════╝
throttle: {:.2f}  steer: {:+.2f}  brake: {:.2f}
"""


def get_key(fd, old):
    tty.setraw(fd)
    ch = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    # Caps clamp the *output* command, not the input step — so W/D still
    # ramp at full STEP rate but the published message saturates at the
    # cap. Lets you "feel" the cap as a ceiling without changing the
    # keystroke cadence. Safe defaults are conservative; pass --max-* to
    # widen for aggressive validation.
    parser = argparse.ArgumentParser(description="IFSSIM keyboard teleop")
    parser.add_argument("--max-throttle", type=float, default=1.0,
                        help="upper bound on published throttle (0–1)")
    parser.add_argument("--max-steer", type=float, default=1.0,
                        help="upper bound on |published steer| (0–1)")
    args = parser.parse_args()
    max_throttle = max(0.0, min(1.0, args.max_throttle))
    max_steer    = max(0.0, min(1.0, args.max_steer))

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub  = node.create_publisher(ControlCommand, '/control_command', 10)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    throttle = 0.0
    steer    = 0.0
    brake    = 0.0

    print(f"caps: throttle ≤ {max_throttle:.2f}, |steer| ≤ {max_steer:.2f}")
    print(HELP.format(throttle, steer, brake))

    try:
        while rclpy.ok():
            key = get_key(fd, old).lower()

            if key == 'q':
                break
            elif key == 'w':
                throttle = min(1.0, throttle + THROTTLE_STEP)
                brake    = 0.0
            elif key == 's':
                brake    = min(1.0, brake + THROTTLE_STEP)
                throttle = 0.0
            elif key == 'a':
                steer = max(-1.0, steer - STEER_STEP)
            elif key == 'd':
                steer = min(1.0, steer + STEER_STEP)
            elif key == ' ':
                throttle = 0.0
                steer    = 0.0
                brake    = 1.0
            else:
                # No key — decay steering back to centre
                steer    = steer * DECAY
                throttle = max(0.0, throttle - 0.02)
                brake    = 0.0

            # Apply caps right before publish (preserves internal state
            # so STEP/DECAY behavior is unchanged).
            pub_throttle = min(throttle, max_throttle)
            if steer >= 0.0:
                pub_steer = min(steer, max_steer)
            else:
                pub_steer = max(steer, -max_steer)
            msg          = ControlCommand()
            msg.throttle = float(pub_throttle)
            msg.steering = float(pub_steer)
            msg.brake    = float(brake)
            pub.publish(msg)

            sys.stdout.write('\r' + f'throttle: {pub_throttle:.2f}  steer: {pub_steer:+.2f}  brake: {brake:.2f}   ')
            sys.stdout.flush()

    finally:
        # Stop the car on exit
        stop = ControlCommand()
        stop.throttle = 0.0
        stop.steering = 0.0
        stop.brake    = 1.0
        pub.publish(stop)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\nStopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
