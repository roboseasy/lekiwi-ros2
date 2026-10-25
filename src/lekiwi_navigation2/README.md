# lekiwi_navigation2

**Pi에서** 저장한 지도로 LeKiwi를 주행하는 Nav2 launch · 설정 패키지. 화면은 PC RViz로 보고 목표를 준다.
저장 지도 + Omni AMCL로 위치를 잡고, MPPI(Omni)로 경로를 따라가며, Collision Monitor를 거친 `cmd_vel`만 베이스 컨트롤러로 보낸다.
LeKiwi 저장소 `lekiwi_navigation2`를 옮겼다. Nav2 설정값은 저장소와 같고 레이저 토픽만 바뀌었다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · Nav2
- 실행 위치: **Pi** (`lekiwi_bringup`과 함께). Pi에는 화면이 없으므로 **PC에서 RViz**(`rviz/nav2.rviz`)를 연다.
- 이름 기준: `docs/interfaces.md`

> ⚠️ **실제로 바퀴가 움직인다.** launch가 모터 감독 도구(`lekiwi_teleop` `lekiwi_nav2_motor_guard`)를 시작하고, 검사를 통과하면 바퀴 토크를 켠다.
> 감독 도구는 **조작 화면(RViz 노드 `rviz` · `rviz2`)이 ROS 그래프에 있을 때만** 토크를 켜고, 모두 사라지면(PC RViz를 닫으면) 끈다.
> 실행 전 이동 경로, 바퀴 정지, 사람 · 케이블, 즉시 전원 차단을 현장에서 확인한다. 지도만 볼 때는 `auto_arm_motors:=false`.
>
> ⚠️ 같은 ROS domain에서 `lekiwi_cartographer`와 **동시에 실행하지 않는다.** 둘 다 `map → odom`을 발행한다.
> Nav2 주행 중 조작 창이나 다른 `cmd_vel` 발행자를 함께 실행하지 않는다.

---

## 설치

**Pi (한 번만)**: Cartographer와 함께 apt로 설치한다. 명령은 [`lekiwi_cartographer` 설치](../lekiwi_cartographer/README.md#설치-pi-한-번만).
rosdep은 쓰지 않는다 (`rviz2` · `nav2_rviz_plugins`까지 깔린다).

**PC (한 번만)**: RViz의 **Nav2 Goal** 도구 · Navigation 패널 · 입자 표시는 `nav2_rviz_plugins`에 있다.

```bash
sudo apt install ros-jazzy-nav2-rviz-plugins
```

---

## 노드

외부 패키지 노드와 `lekiwi_teleop`의 감독 도구를 실행한다. 이 패키지는 설정과 launch만 가진다.

| 노드 | 패키지 | 하는 일 |
|------|--------|---------|
| `map_server` | `nav2_map_server` | 저장 지도 (`map:=`) → `/map` |
| `amcl` | `nav2_amcl` | Omni 모델 위치 추정 → TF `map → odom` |
| `controller_server` | `nav2_controller` | MPPI (Omni) → `cmd_vel_nav` |
| `planner_server` | `nav2_planner` | NavFn 전역 경로 |
| `smoother_server` | `nav2_smoother` | 경로 다듬기 |
| `behavior_server` | `nav2_behaviors` | spin · backup · wait → `cmd_vel_nav` |
| `bt_navigator`, `waypoint_follower` | `nav2_bt_navigator`, `nav2_waypoint_follower` | 목표 처리 |
| `velocity_smoother` | `nav2_velocity_smoother` | `cmd_vel_nav` → 속도 · 가속 제한 → `cmd_vel_smoothed` |
| `collision_monitor` | `nav2_collision_monitor` | `cmd_vel_smoothed` → 충돌 감시 → **`cmd_vel` (유일한 발행자)** |
| `lifecycle_manager_localization`, `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | 자동 활성화 |
| `rviz2` | `rviz2` | `rviz/nav2.rviz` (`use_rviz:=true`일 때. 화면이 있는 컴퓨터에서만) |
| `lekiwi_nav2_motor_guard` | `lekiwi_teleop` | 조작 화면(RViz)이 있을 때 `lekiwi/motor/power`로 토크를 켜고, 상태가 나빠지거나 RViz가 닫히거나 시간이 끝나면 끈다 |

## 구독 (Subscribe)

| 토픽 | 타입 | 받는 노드 | 내용 |
|------|------|-----------|------|
| `/lekiwi/sensor/scan_filtered` | `sensor_msgs/LaserScan` | `amcl`, local · global costmap, `collision_monitor` | Pi `lekiwi_scan_filter` 출력. 제외한 빔은 NaN (`inf_is_valid: false`) |
| `/odom` | `nav_msgs/Odometry` | `bt_navigator`, `velocity_smoother` 등 | 베이스 컨트롤러 엔코더 odom |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 전체 | `odom → base_footprint → … → lidar_link` |
| `/initialpose`, `/goal_pose` | `geometry_msgs/PoseWithCovarianceStamped`, `PoseStamped` | `amcl`, `bt_navigator` | RViz **2D Pose Estimate** · **Nav2 Goal** |

## 발행 (Publish)

| 토픽 | 타입 | 발행 노드 | 내용 |
|------|------|-----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | `collision_monitor` | Pi `lekiwi_base_controller_node`로 가는 최종 명령 |
| `/tf` (`map → odom`) | `tf2_msgs/TFMessage` | `amcl` | 초기 위치를 지정한 뒤부터 |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` | 저장 지도 |
| `/plan`, `/local_costmap/…`, `/global_costmap/…`, `/particle_cloud` | – | Nav2 | RViz 표시용 |

**명령 경로**

```text
controller_server · behavior_server ─ cmd_vel_nav ─▶ velocity_smoother ─ cmd_vel_smoothed ─▶ collision_monitor ─ cmd_vel ─▶ lekiwi_base_controller_node
```

`/cmd_vel`에 직접 보낸 명령은 이 충돌 감시 경로를 거치지 않는다.

## 파라미터

**launch 인자** (`launch/navigation.launch.py`)

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `map` | `~/maps/lekiwi/map.yaml` | 저장 지도 YAML. 없으면 Nav2를 띄우기 전에 오류로 멈춘다 |
| `params_file` | `share/lekiwi_navigation2/config/nav2_params.yaml` | Nav2 파라미터 |
| `autostart` | `true` | Nav2 자동 활성화 |
| `localization_autostart` | `true` | map_server · AMCL 자동 활성화 |
| `use_rviz` | `false` | 이 컴퓨터에서 RViz를 함께 실행. Pi에는 화면이 없어 끈다. `true`이면 그 RViz를 닫을 때 launch 전체가 끝난다 |
| `auto_arm_motors` | `true` | 모터 감독 도구 시작 |
| `operator_nodes` | `rviz,rviz2` | 감독 도구가 지켜보는 조작 화면 노드 이름. 하나라도 있어야 토크를 켜고, 모두 사라지면 끈다. PC에서 `rviz2 -d`로 열면 이름은 `rviz`, launch 파일로 열면 `rviz2` (2026-10-18 lekiwi04 · PC에서 `ros2 node list`로 확인) |
| `motor_guard_duration` | `3600` | [s] 감독 도구 최대 실행 시간 |
| `use_respawn` | `false` | Nav2 노드 자동 재시작 |
| `log_level` | `info` | Nav2 로그 수준 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |

**주요 설정** (`config/nav2_params.yaml`)

| 항목 | 값 | 비고 |
|------|----|------|
| AMCL | `nav2_amcl::OmniMotionModel`, 입자 1000 ~ 5000, 빔 120, 거리 0.19 ~ 6.0 m | `set_initial_pose: false` — 초기 위치는 RViz에서 직접 지정 |
| MPPI | `motion_model: Omni`, 5 Hz × `model_dt` 0.2, 400 samples × 35 steps | `PreferForwardCritic` 없음 (횡이동 허용) |
| 속도 상한 (velocity_smoother) | x 0.08 m/s, y 0.06 m/s, 회전 0.25 rad/s | 저장소 차체 상한 x 0.11 · y 0.10 · 평면 0.11 m/s · 0.75 rad/s 이내 (베이스 컨트롤러 상한은 2026-10-22부터 0.23 m/s · 1.53 rad/s) |
| costmap | 반경 0.22 m, inflation 0.35 m, 장애물 0.19 ~ 3.0 m, `expected_update_rate` 0.25 s | `robot_base_frame: base_footprint`, `sensor_frame: lidar_link` |
| Collision Monitor | `FootprintApproach` 1.5 s, `source_timeout` 1.0 s | 입력 `cmd_vel_smoothed` → 출력 `cmd_vel` |
| `enable_stamped_cmd_vel` | `false` | 베이스 컨트롤러는 `geometry_msgs/Twist`를 받는다 |

거리 하한 0.19 m는 `lekiwi_scan_filter`의 `self_radius`와 같다 (테스트가 확인한다).

---

## 빌드

감독 도구(`lekiwi_teleop`)를 함께 빌드한다.

```bash
cd ~/lekiwi_ws        # Pi 워크스페이스
cbs lekiwi_teleop lekiwi_navigation2 && sb
```

PC는 RViz 설정 파일(`src/lekiwi_navigation2/rviz/nav2.rviz`)을 작업 사본에서 바로 연다. 로봇 모델을 보려고 `lekiwi_description`만 빌드한다.

**테스트** (Nav2 없이 파라미터 · launch · RViz 계약 검사)

```bash
colcon test --packages-select lekiwi_navigation2 --event-handlers console_direct+
colcon test-result --verbose
```

---

## 실행

1. **Pi 터미널 1**: 센서 · 베이스 bringup (토크 꺼진 상태). Cartographer와 키보드 조작 창은 종료한다.
2. **PC**: Pi와 같은 `ROS_DOMAIN_ID`(LeKiwi 기본 42) · `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`로 **RViz를 먼저 연다.**
   감독 도구는 이 RViz가 보여야 토크를 켠다 (시작 후 60초 안).

   ```bash
   rviz2 -d $LEKIWI_WS/src/lekiwi_navigation2/rviz/nav2.rviz   # LEKIWI_WS: 저장소 README 1-②
   ```

3. **Pi 터미널 2**: 저장 지도로 시작한다.

   ```bash
   ros2 launch lekiwi_navigation2 navigation.launch.py
   # 다른 지도 (save_map.sh가 출력한 경로)
   ros2 launch lekiwi_navigation2 navigation.launch.py map:=$HOME/maps/lekiwi/map_<날짜_시각>.yaml
   # 지도 · 위치 추정만 보기 (토크를 켜지 않는다)
   ros2 launch lekiwi_navigation2 navigation.launch.py auto_arm_motors:=false
   ```

4. RViz **2D Pose Estimate**로 실제 위치 · 방향을 지정한다. 레이저 점이 지도 벽과 맞고 `map → odom` TF가 생겼는지 확인한다.
   전역 costmap(`planner_server`)은 이 위치가 생길 때까지 활성화를 기다린다 (`initial_transform_timeout` 600 s). 주행 쪽 `Managed nodes are active`는 이때 나온다.
   초기 위치를 (0, 0)으로 가정하지 않는다. RViz의 **Startup** 버튼은 누르지 않는다 (이미 활성화된 노드를 다시 시작한다).
5. 터미널의 `Nav2 motor guard active`와 `/lekiwi/motor/ready = true`를 확인한 뒤, RViz **Nav2 Goal**로 짧은 목표 하나를 지정한다.
   `/lekiwi/motor/ready = false`이면 목표를 보내지 않고 베이스의 고장 원인을 확인한다.
6. 끝낼 때: 진행 중인 목표를 취소하고 실제 정지를 확인한 뒤 PC RViz를 닫거나 Pi 터미널 2에서 `Ctrl+C`. 감독 도구가 토크를 끈다.
   서비스 응답이나 프로세스 종료만으로 바퀴 정지를 가정하지 않는다.

---

## 확인 명령어

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic hz /lekiwi/sensor/scan_filtered` | 스캔이 오는지 (4 Hz 이상이어야 costmap이 멈추지 않는다) |
| `ros2 topic info /cmd_vel -v` | 발행자가 `collision_monitor` 하나, 구독자가 `lekiwi_base_controller_node` 하나인지 |
| `ros2 topic echo /lekiwi/motor/ready --once --qos-reliability reliable --qos-durability transient_local` | 모터 준비 상태 |
| `ros2 run tf2_ros tf2_echo map odom` | AMCL이 `map → odom`을 내는지 (초기 위치 지정 후) |
| `ros2 lifecycle get /controller_server` | `active`인지 |
| `ros2 topic echo /collision_monitor_state` | 충돌 감시 동작 상태 |

---

## 알려진 한계

- 실물 주행은 좁은 지도에서 1 m 안쪽 목표만 확인했다. PC RViz를 닫았을 때 토크가 꺼지는 것, 충돌 감시 정지는 아직 확인하지 않았다.
- Pi CPU 여유가 크지 않다. bringup(관절 50 Hz일 때)과 함께 목표 없이 전체 약 65%였다. 이때 Pi에서 ros2 CLI(`ros2 lifecycle get` 등)를
  연달아 실행하자 부하가 8.7까지 올라 `behavior_server` heartbeat가 4 s 끊겼고, lifecycle manager가 Nav2를 내렸다.
  **주행 중 확인 명령은 PC에서 실행한다.** bringup 관절 발행은 10 Hz로 낮췄다 (`joint_state_rate`).
- 감독 도구는 이름이 `rviz` · `rviz2`인 노드를 어느 컴퓨터에서든 조작 화면으로 친다. 다른 RViz(지도 작성용 등)가 떠 있으면 Nav2 RViz를 닫아도 토크가 켜져 있다.
  Nav2 때는 RViz를 하나만 연다. Wi-Fi가 끊기면 RViz가 사라졌다고 판단하기까지 최대 약 20 s 걸린다.
- 감독 도구는 한 번 토크를 끄면 다시 켜지 않는다. RViz를 다시 열었으면 Nav2 launch를 다시 시작한다.
- 하부에 거꾸로 단 라이다는 바퀴 부근이 가려진다. Collision Monitor가 모든 장애물을 보는 것은 아니다.
- 반경 0.22 m는 팔을 포함한 실제 점유 면적과 현장에서 비교해야 한다.
- 여러 축을 동시에 명령하면 바퀴 상한 때문에 속도가 비례 축소될 수 있다.
- 스캔 `time_increment = 0`이라 이동 중 스캔 왜곡을 보정하지 않는다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `Map file does not exist: …` | 지도를 먼저 저장 (`ros2 run lekiwi_cartographer save_map.sh`) 또는 `map:=`로 지정 |
| `Package 'nav2_…' not found` | Pi에 Nav2가 없음. [설치](#설치) 실행 |
| `Package 'lekiwi_teleop' not found` | 감독 도구를 빌드하지 않음. `cbs lekiwi_teleop` (또는 `auto_arm_motors:=false`) |
| costmap에 장애물이 안 생김 / `sensor origin out of map` | `/lekiwi/sensor/scan_filtered` 수신, TF `base_footprint → lidar_link` 확인 |
| `expected update rate` 경고 후 멈춤 | 스캔이 0.25 s 넘게 끊김. Pi 라이다 · 필터 · Wi-Fi 확인 |
| 감독 도구가 `expected only /collision_monitor on cmd_vel` | 조작 창 등 다른 `cmd_vel` 발행자를 종료 |
| 감독 도구가 `no operator view (rviz, rviz2) is running` | PC RViz를 먼저 열고 Nav2를 다시 시작. PC와 Pi의 `ROS_DOMAIN_ID` 확인 |
| `Timed out waiting for transform from base_footprint to odom`가 계속 나오고, 감독 도구가 `fresh lekiwi/sensor/scan_filtered and odom were not observed`로 끝남 | **bringup이 꺼져 있다** (2026-10-18 실제로 겪음). Nav2를 `Ctrl+C` → bringup을 켜고 → Nav2를 다시 시작. 감독 도구는 이때 토크를 켜지 않는다 |
| PC RViz에 Nav2 Goal 도구가 없음 / `nav2_rviz_plugins` 오류 | PC에 `ros-jazzy-nav2-rviz-plugins` 설치 |
| 로봇이 목표로 가지 않음 | 초기 위치 지정 여부, `/lekiwi/motor/ready`, lifecycle 상태 확인 |
| `Timed out waiting for transform from base_footprint to map`가 계속 나옴 | 정상. RViz에서 초기 위치를 찍으면 멈춘다. 10분 안에 찍지 않으면 Nav2를 다시 시작한다 |

---

## 패키지 구조

```text
lekiwi_navigation2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/nav2_params.yaml          # Nav2 파라미터
├── launch/navigation.launch.py      # 저장 지도 주행 + 감독 도구 (+ RViz, 화면이 있을 때만)
├── rviz/nav2.rviz
└── test/test_navigation_contract.py
```

## 참고한 코드

- **LeKiwi 저장소 `lekiwi_navigation2`**: 파라미터, launch, RViz 설정, 계약 테스트
- **LeKiwi 저장소 `docs/hardware/slam_navigation.md`**: 속도 상한 근거, 실행 · 정지 순서, 검증 구분
- **설정 출처**: `yeon-03/lekiwi-pill-pickup` commit `9f4a00cfd9dd15e31e73f96825eb86193ef2a20a`. 조건은 워크스페이스 `THIRD_PARTY_NOTICES.md`

저장소와 다른 점:

- 스캔 필터를 띄우지 않는다 (`use_scan_filter` 인자 삭제). Pi가 `/lekiwi/sensor/scan_filtered`를 발행한다.
- 레이저 입력 4곳(AMCL `scan_topic`, local · global costmap `scan.topic`, Collision Monitor `scan.topic`): `/scan_navigation` → `/lekiwi/sensor/scan_filtered`
- 그 밖의 노드 · 파라미터 · 인자 기본값은 같다. 감독 도구가 확인하는 베이스 노드 · 모터 토픽 이름은 `lekiwi_teleop`이 정한다.

## 검증 기록

- 2026-10-18, PC (Ubuntu 24.04, Jazzy, Nav2 미설치): 빌드 성공. 계약 테스트 11개 통과
  (워크스페이스 `lekiwi_base_controller` 설정의 속도 상한과 `lekiwi_scan_filter`의 `self_radius` 대조 포함)
- 가짜 `nav2_common`으로 launch 설명 생성 확인 (`--show-args`), 노드 정의가 저장소 launch와 같음을 AST로 비교, YAML 파싱 확인
- 2026-10-18, lekiwi04 (Pi 4, Jazzy, Nav2 1.3.13 apt), bringup(관절 50 Hz)과 함께, `auto_arm_motors:=false`, **목표 없음 · 로봇 정지**:
  - 빌드 성공, 계약 테스트 12개 통과
  - 그 자리에서 만든 47 × 53칸 지도로 기동: 모든 플러그인 적재, `lifecycle_manager_localization` · `_navigation` 모두 `Managed nodes are active`
  - `/initialpose`를 준 뒤 AMCL이 `map → odom` 발행, 전역 costmap 활성화
  - 90초 동안 heartbeat 끊김 없음. CPU 전체 약 65%, Nav2 노드마다 10 ~ 25% (`/tf` 약 67 Hz)
- 2026-10-18 21:30 ~ 21:35, lekiwi04, **사람 있음**: bringup(관절 10 Hz) + Nav2(Pi) + PC RViz(`rviz2 -d`, 노드 `/rviz`), 지도는 `lekiwi_cartographer`로 만든 120 × 104칸
  - 감독 도구가 PC RViz를 보고 토크를 켬. 초기 위치 후 주행 쪽 `Managed nodes are active`
  - 목표 3개: 0.49 m를 12 s에 `Goal succeeded`, 하나는 새 목표로 대체, 0.75 m를 15 s에 `Goal succeeded` (`Reached the goal!`)
  - 같은 날 찾아서 고친 것: 초기 위치를 60 s 안에 찍지 않으면 주행 쪽 활성화 실패 → 전역 costmap `initial_transform_timeout` 600 s
  - 의도대로 동작한 안전 처리: bringup이 꺼진 채 시작 → `fresh ... odom were not observed`로 토크를 켜지 않음. RViz 없이 시작 → 토크를 켜지 않고 감독 도구 종료
- PC RViz를 닫았을 때 토크 끔, 충돌 감시 정지, 넓은 공간 주행은 아직 검증하지 않았다.
