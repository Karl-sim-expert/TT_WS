#!/usr/bin/env bash
# Kill leftover simulation processes (Gazebo, bridges, RViz2, spawners) and reset the ROS 2 daemon.
for pat in "ign gazebo" "ign-gazebo" "gz sim" parameter_bridge robot_state_publisher \
           rviz2 "controller_manager/spawner" "spawner" scene_markers.py; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
ros2 daemon stop 2>/dev/null || true
sleep 1
left=$(ps aux | grep -E "ign gazebo|ign-gazebo|gz sim|parameter_bridge|robot_state_publisher|rviz2|scene_markers" | grep -v grep | grep -v kill_sim)
if [ -z "$left" ]; then echo "clean: no simulation processes left"; else echo "still running:"; echo "$left"; fi
