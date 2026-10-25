# Stroke decision: when and how to hit, and where the ball should land.
uint8 TOPSPIN_DRIVE=0
uint8 LOOP=1
uint8 BLOCK=2
uint8 BACKSPIN_CHOP=3

std_msgs/Header header
builtin_interfaces/Time hit_time
geometry_msgs/Pose paddle_pose          # paddle pose at impact (normal = local +z)
geometry_msgs/Twist paddle_twist        # paddle velocity at impact
geometry_msgs/Point target_landing      # desired landing point on opponent half [m]
uint8 stroke_type
