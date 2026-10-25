# Predicted ball state at the chosen hitting time t*.
std_msgs/Header header
builtin_interfaces/Time hit_time
geometry_msgs/Point hit_position        # [m]
geometry_msgs/Vector3 velocity_in       # [m/s]
geometry_msgs/Vector3 angular_velocity_in  # [rad/s]
float64 confidence                      # 0..1
