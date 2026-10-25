# tt_ws

ROS 2 Humble 기반 탁구 로봇 워크스페이스 (Franka Panda, Gazebo 시뮬레이션 전용).

```bash
cd ~/tt_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

패키지 구성과 설계는 설계 문서(7절 이후 확정 조건 포함)를 따른다. 현재 포함된 패키지는 `src/` 아래를 참고.

## 실행 (WSL, Humble + Gazebo Fortress)

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers ros-humble-ros2controlcli ros-humble-moveit-resources-panda-description ros-humble-xacro
colcon build --symlink-install && source install/setup.bash
ros2 launch tt_bringup sim.launch.py            # GUI 포함
ros2 launch tt_bringup sim.launch.py gz_args:="-r -s --render-engine ogre"   # 서버만
```

### 가벼운 화면 (RViz2)
Gazebo 창 대신 RViz2로 로봇·탁구대·공을 본다 (Gazebo는 GUI 없이 서버만):
```bash
sudo apt install ros-humble-rviz2
ros2 launch tt_bringup sim.launch.py gz_args:="-r -s --render-engine ogre" rviz:=true
```
