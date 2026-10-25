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
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros2controlcli ros-humble-moveit-resources-panda-description ros-humble-xacro ros-humble-rviz2
cd ~/tt_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch tt_bringup sim.launch.py
```

기본값: Gazebo는 GUI 없이 서버만 돌고(물리 시뮬레이션), 화면은 RViz2로 본다.

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `rviz` | `true` | RViz2 화면 (로봇, 탁구대, 공) |
| `cameras` | `false` | RGB 카메라 2대 (120 Hz). 켜면 RTF가 크게 떨어짐 |
| `gz_args` | `-r -s --render-engine ogre` | Gazebo 인자. GUI가 필요하면 `gz_args:="-r --render-engine ogre"` |

### 종료와 정리
- 정상 종료: 실행 터미널에서 `Ctrl+C` **한 번**만 누르고 5초쯤 기다리거나, RViz2 창을 닫는다 (창을 닫으면 전체가 같이 종료됨).
- 잔여 프로세스 정리: `~/tt_ws/tools/kill_sim.sh` (`clean: ...`이 나오면 정상)

### 공 발사 (볼머신 대용)
시뮬레이션이 켜진 상태에서 다른 터미널에서 실행:
```bash
~/tt_ws/tools/fire_ball.sh                       # 8 m/s, 스핀 없음
~/tt_ws/tools/fire_ball.sh 8 0 1 0 300 0         # 톱스핀 (+y 회전 = 전진 구름)
~/tt_ws/tools/fire_ball.sh 8 0 1 0 -300 0        # 백스핀
~/tt_ws/tools/fire_ball.sh 8 0 1 0 0 300         # 사이드스핀
```
인자: `vx vy vz [wx wy wz]` (속도 m/s, 스핀 rad/s; 100 rps = 628 rad/s)
공 물리(항력, 마그누스, 탁구대 바운스)는 `tt_gazebo`의 `BallPhysics` 플러그인이 계산한다.
