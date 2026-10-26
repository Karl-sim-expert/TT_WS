# tt_ws

ROS 2 Humble 기반 탁구 로봇 워크스페이스 (Franka Panda, Gazebo 시뮬레이션 전용).

진행 상황: M0(환경) 완료, M1(탁구대·공 물리, 볼머신) 완료. M2(팔 제어, 정지한 공 때리기) 코드 작성 완료, WSL 테스트 중.

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

### 볼머신 (랜덤 낙하점 자동 발사)
```bash
ros2 launch tt_bringup sim.launch.py ball_machine:=true machine_period:=3.0 machine_miss_percent:=20 machine_speed:=9.0 machine_spin_max:=200
```
매 발마다 로봇 쪽 판 위의 랜덤 지점(x: `machine_x_min`~1.25, y: ±0.7)을 정해 그곳에 떨어지도록 발사 각도를 자동 계산한다.
`machine_miss_percent`(%)의 확률로 판 밖(끝선 너머 또는 옆선 밖)을 겨냥한다. 스핀은 매 발 랜덤이다: 윗/백스핀은 ±`machine_spin_max`, 옆스핀은 ±`machine_spin_max`/2 (rad/s, 0이면 스핀 없음).
빠른 공은 네트 바로 너머에 못 떨어뜨리므로 `machine_x_min` 기본값은 0.6이다. 계산만 확인: `python3 src/tt_bringup/scripts/ball_machine.py --solve 9 0.8 0 100 0`

### 공 발사 (수동)
시뮬레이션이 켜진 상태에서 다른 터미널에서 실행:
```bash
~/tt_ws/tools/fire_ball.sh                       # 8 m/s, 스핀 없음
~/tt_ws/tools/fire_ball.sh 8 0 1 0 300 0         # 톱스핀 (+y 회전 = 전진 구름)
~/tt_ws/tools/fire_ball.sh 8 0 1 0 -300 0        # 백스핀
~/tt_ws/tools/fire_ball.sh 8 0 1 0 0 300         # 사이드스핀
```
인자: `vx vy vz [wx wy wz]` (속도 m/s, 스핀 rad/s; 100 rps = 628 rad/s)
공 물리(항력, 마그누스, 탁구대 바운스)는 `tt_gazebo`의 `BallPhysics` 플러그인이 계산한다.

## M2: 정지한 공 때리기

로봇은 Panda 팔이 **타구 방향(x축) 레일** 위에 올라간 구조다 (`rail_joint`, 범위 -0.5~0.6 m, 속도 4 m/s). 레일이 빠른 직선 이동을, 팔이 라켓 각도를 맡는다.
Panda만 쓰면 정면 타구 시 공 속도가 1.2 m/s가 한계였고, 레일을 달면 오프라인 계산으로 평평한 타구 약 6.9 m/s, 위로 25° 약 5.9 m/s까지 가능하다 (위로 40°는 약 3.4 m/s, 팔 관절이 병목).

설치(최초 1회): `sudo apt install ros-humble-moveit`

```bash
cd ~/tt_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch tt_bringup static_hit.launch.py
```
공이 허공에 멈춰 있고(`/ball/hold`), 로봇이 `StrokePlan`에 맞춰 스윙해 지정한 방향으로 공을 날린다. 끝나면 터미널에
`HIT: timing error ... ms, direction error ... deg`가 출력된다(타이밍 오차 목표 ≤ 10 ms).

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `speed` | `5.0` | 요청하는 공 속도 [m/s] (관절 속도 한계에 걸리면 자동으로 낮춘다) |
| `elevation_deg` / `yaw_deg` | `25` / `0` | 공을 보낼 방향 (위로 각도 / 좌우 각도) |
| `ball_x` `ball_y` `ball_z` | `1.15 0 0.95` | 공을 멈춰 둘 위치 [m] |
| `vel_scale` | `1.0` | 모든 관절(레일 4 m/s, Panda 2.4~2.9 rad/s) 속도 한계 배율. 1보다 크면 시뮬레이션 전용 가상 팔 |
| `repeat` | `1` | 반복 횟수 (끝에 평균/최대 오차 요약) |
| `lead` | `4.0` | 계획 후 타격까지 시간 [s] |

예: `ros2 launch tt_bringup static_hit.launch.py speed:=3.0 elevation_deg:=30 repeat:=5`

공 위치를 매 반복마다 무작위로: `ros2 launch tt_bringup static_hit.launch.py repeat:=10 random_pos:=true`
(기본 범위 x∈[1.0,1.4], y∈[-0.7,0.7] (Y 레일로 탁구대 폭 전체), z∈[0.85,1.15] — 5 m/s 타격이 도달 가능한 영역. `seed:=42`로 재현,
`x_min`/`x_max`/`y_min`/… 로 범위 변경)

- 공은 시작할 때 볼머신 위치에 멈춰 있다가 `fire_ball.sh` / 볼머신이 쏠 때만 움직인다. 위치 지정: `tools/hold_ball.sh 1.15 0 0.95`
- 라켓-공 충돌은 엔진 접촉이 아니라 `BallPhysics` 플러그인이 해석적으로 계산한다 (반발계수 0.85, 마찰 0.45).
- 라켓에 맞을 때마다 `/ball/paddle_hit`(nav_msgs/Odometry)가 나온다: 시각, 위치, 맞은 직후 속도·스핀.
- MoveIt (선택): `ros2 launch tt_bringup sim.launch.py moveit:=true` 후 `ros2 launch tt_moveit_config move_group.launch.py rviz:=true` (RViz2 MotionPlanning).
  고속 스윙은 `tt_motion`이 직접 만들고, MoveIt은 설정·IK·충돌 검사·느린 이동에 쓴다 (설계 문서 5절).
- 클라우드 없이 계산만 검증: `python3 -m unittest discover -s src/tt_motion/test` (FK/IK, 스윙 궤적, 관절 한계)

## M3: 날아오는 공 되받아치기 (초안, 정답 공 위치 사용 — 비전은 아직 없음)

```bash
cd ~/tt_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch tt_bringup return_ball.launch.py
```
볼머신이 공을 쏘면(`machine_period` 초마다) `return_ball` 노드가 공이 출발한 순간의 위치·속도로 비행(공기저항, 스핀, 바운스)을 예측하고,
바운스 뒤 라켓이 닿을 수 있는 타점과 시각, 라켓 방향·속도를 정해 `StrokePlan`을 한 번 보낸다(M2 `stroke_executor`가 그대로 스윙).
돌려보낼 공은 상대 코트 `target_x`/`target_y`(기본 -0.7, 0)에 떨어지도록 속도를 계산한다(네트 통과 확인 포함).
터미널에 샷마다 아래가 나온다.
- `shot N: ... hit at ... in X s` — 계획(타점, 남은 시간, 라켓 속도, 예상 낙하점)
- `HIT k/N: timing error ... ms` — 라켓이 공을 친 시각 오차
- `RESULT: landed at (...), error ... cm, ON opponent half | returned R/N shots` — 돌려보낸 공의 실제 낙하점
- `cannot return (...)` — 이 공은 도달 불가라 포기(이유 표시), `MISS: ...` — 헛스윙

옵션: `machine_speed`(기본 6.0, 6 m/s 미만은 네트를 넘는 샷이 없음), `machine_period`(5.0), `machine_spin_max`(0 = 스핀 없음), `target_x`, `target_y`, `min_lead`.
오프라인 검증(ROS 불필요): `python3 -m unittest discover -s src/tt_motion/test`
