#!/usr/bin/env bash
# Fire the ball from the machine position. Usage: fire_ball.sh [vx vy vz [wx wy wz]]
#   velocity in m/s, spin in rad/s (100 rps = 628 rad/s). Defaults: 8 m/s forward, no spin.
vx=${1:-8.0}; vy=${2:-0.0}; vz=${3:-1.0}
wx=${4:-0.0}; wy=${5:-0.0}; wz=${6:-0.0}
ros2 topic pub --once /ball/launch geometry_msgs/msg/Twist \
  "{linear: {x: $vx, y: $vy, z: $vz}, angular: {x: $wx, y: $wy, z: $wz}}"
