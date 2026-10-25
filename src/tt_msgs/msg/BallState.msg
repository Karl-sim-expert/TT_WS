# Ball state in the world frame. Published by the Gazebo ground-truth plugin
# (/ball/state_gt) or by the estimator (/ball/state).
std_msgs/Header header
geometry_msgs/Point position            # [m]
geometry_msgs/Vector3 velocity          # [m/s]
geometry_msgs/Vector3 angular_velocity  # [rad/s]
# Row-major 9x9 covariance of [position, velocity, angular_velocity].
# All zeros for ground truth.
float64[81] covariance
