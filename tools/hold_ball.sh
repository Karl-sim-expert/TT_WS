#!/usr/bin/env bash
# Park the ball motionless in the air (no gravity) at a point until it is hit or launched.
# Usage: hold_ball.sh [x y z]     default: 1.15 0 0.95 (just in front of the paddle's ready pose)
x=${1:-1.15}; y=${2:-0.0}; z=${3:-0.95}
ros2 topic pub --once /ball/hold geometry_msgs/msg/Vector3 "{x: $x, y: $y, z: $z}"
