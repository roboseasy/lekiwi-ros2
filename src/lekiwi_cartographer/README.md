# lekiwi_cartographer

**Pi에서** bringup이 내는 필터된 라이다 스캔 · IMU · odom으로 **Cartographer 2D 지도를 만들고 저장**하는 launch · 설정 패키지.
LeKiwi 저장소 `lekiwi_cartographer`와 `tools/save_map.sh`를 옮겼다. 지도 설정값은 저장소와 같고, 최적화 스레드만 4 → 2로 줄였다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · `cartographer_ros`
- 실행 위치: **Pi** (`lekiwi_bringup`과 함께). Pi에는 화면이 없으므로 **지도는 PC RViz로 본다** (`rviz/slam.rviz`).
- 입력 이름: `docs/interfaces.md`

> ⚠️ 같은 ROS domain에서 `lekiwi_navigation2`(AMCL)와 **동시에 실행하지 않는다.** 둘 다 `map → odom`을 발행한다.
>
> ✅ 스캔 필터는 이 launch에서 띄우지 않는다. bringup의 `lekiwi_scan_filter` 출력(`/lekiwi/sensor/scan_filtered`)을 그대로 받는다.

---

## 설치 (Pi, 한 번만)

rosdep은 쓰지 않는다. `package.xml`의 `rviz2` · `nav2_rviz_plugins`까지 깔려 화면 없는 Pi에 쓸모없는 Qt 패키지가 늘어난다.
실행에 필요한 패키지만 apt로 설치한다 (Nav2 패키지도 함께. 약 400개, 2~3 GB).

```bash
sudo apt install --no-install-recommends ros-jazzy-cartographer-ros \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl ros-jazzy-nav2-behaviors ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-common ros-jazzy-nav2-controller ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-navfn-planner \
  ros-jazzy-nav2-planner ros-jazzy-nav2-smoother ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-waypoint-follower
```

PC에는 설치하지 않는다. PC는 RViz로 보기만 한다.

---

## 노드

외부 패키지 노드를 실행한다. 이 패키지는 설정과 launch만 가진다.

- `cartographer_node` (`cartographer_ros`)
- `cartographer_occupancy_grid_node` (`cartographer_ros`)
- `rviz2` (`use_rviz:=true`일 때. 화면이 있는 컴퓨터에서만)

## 구독 (Subscribe)

| 토픽 | 타입 | 노드 | 내용 |
|------|------|------|------|
| `/lekiwi/sensor/scan_filtered` (→ `scan`) | `sensor_msgs/LaserScan` | `cartographer_node` | Pi `lekiwi_scan_filter` 출력. NaN 빔은 관측 없음으로 버려진다 |
| `/lekiwi/sensor/offset_imu` (→ `imu`) | `sensor_msgs/Imu` | `cartographer_node` | `lekiwi_2d_imu.lua`일 때만. 자이로 바이어스만 제거한 값 |
| `/odom` | `nav_msgs/Odometry` | `cartographer_node` | 베이스 컨트롤러의 엔코더 odom (이름 그대로) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `cartographer_node` | `odom → base_footprint → … → lidar_link`, `imu_link` |

## 발행 (Publish)

| 토픽 | 타입 | 노드 | 내용 |
|------|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | `cartographer_occupancy_grid_node` | 칸 0.05 m, 1초마다 |
| `/tf` (`map → odom`) | `tf2_msgs/TFMessage` | `cartographer_node` | `odom → base_footprint`는 발행하지 않는다 (`provide_odom_frame = false`) |
| `/submap_list`, `/trajectory_node_list` 등 | – | `cartographer_node` | Cartographer 기본 출력 |

## 서비스

| 이름 | 타입 | 서버 | 쓰는 곳 |
|------|------|------|---------|
| `/write_state` | `cartographer_ros_msgs/srv/WriteState` | `cartographer_node` | `save_map.sh`가 pbstream 저장에 쓴다 |

## 파라미터

**launch 인자** (`launch/cartographer.launch.py`)

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `configuration_basename` | `lekiwi_2d_imu.lua` | 표준 조립품(BMI160). IMU가 없는 장비는 `lekiwi_2d.lua` |
| `cartographer_config_dir` | `share/lekiwi_cartographer/config` | Lua 설정 폴더 |
| `scan_topic` | `lekiwi/sensor/scan_filtered` | Cartographer `scan`에 연결할 토픽 |
| `imu_topic` | `lekiwi/sensor/offset_imu` | Cartographer `imu`에 연결할 토픽 |
| `resolution` | `0.05` | [m] OccupancyGrid 칸 크기 |
| `publish_period_sec` | `1.0` | [s] OccupancyGrid 발행 주기 |
| `use_rviz` | `false` | 이 컴퓨터에서 RViz(`rviz/slam.rviz`: Fixed Frame `map`, Map `/map`, LaserScan `/lekiwi/sensor/scan_filtered`)를 함께 띄운다. Pi에는 화면이 없어 끈다 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |

**Lua 설정** (`config/`)

| 파일 | `tracking_frame` | `use_imu_data` | 쓰는 경우 |
|------|------------------|----------------|-----------|
| `lekiwi_2d_imu.lua` | `imu_link` | `true` | 기본. BMI160이 있는 표준 조립품 |
| `lekiwi_2d.lua` | `base_footprint` | `false` | IMU가 없는 장비 |

두 파일의 나머지 값은 같다: `published_frame = "odom"`, `provide_odom_frame = false`, `use_odometry = true`,
거리 0.05 ~ 9.0 m, `missing_data_ray_length` 3 m, motion filter 1° · 0.1 m · 2 s, 서브맵 45 scan,
Ceres 가중치 20 · 10 · 40, constraint `min_score` 0.65 · 0.7, 최적화 스레드 2 (Pi 코어 4개 중 2개는 모터 · 센서 노드 몫).

## 지도 저장: `save_map.sh`

```bash
ros2 run lekiwi_cartographer save_map.sh
```

Cartographer를 **유지한 채**, 로봇을 정지하고 **Pi**의 새 터미널에서 실행한다. pbstream은 `cartographer_node`가 자기 컴퓨터에 쓰므로 Cartographer와 같은 컴퓨터에서 실행해야 한다.

| 순서 | 내용 |
|------|------|
| 1. ROS 환경 확인 | `ROS_DISTRO`와 `ros2`가 없거나 `nav2_map_server` · `cartographer_ros_msgs`가 없으면 아무 파일도 만들지 않고 멈춘다 |
| 2. 폴더 · 잠금 | `LEKIWI_MAP_DIR` (설정하지 않으면 `~/maps/lekiwi`, Nav2 기본 지도와 같은 곳). 빈 값이면 멈춘다. 동시에 두 번 실행하지 못하게 잠근다 |
| 3. 이름 | `map`. 같은 이름의 파일(깨진 링크 포함)이 있으면 덮어쓰지 않고 `map_<날짜_시각_ns>`를 쓴다 |
| 4. YAML · PGM | `map_saver_cli` (AMCL용 지도, 10초 제한) |
| 5. pbstream | `/write_state` (Cartographer 상태, 60초 제한). 응답의 `StatusResponse(code=0, …)`까지 확인한다 |
| 6. 확인 · 기록 | 세 파일이 비어 있지 않으면 `last_map.txt`를 한 번에 바꾸고 Nav2 실행 명령을 출력한다 |

중간에 실패하면 단계 이름과 함께 멈추고 지난 `last_map.txt`를 바꾸지 않는다. 이미 저장된 YAML · PGM은 남는다.

```text
Nav2에서 사용할 지도: /home/<사용자>/maps/lekiwi/map.yaml
실행: ros2 launch lekiwi_navigation2 navigation.launch.py map:=/home/<사용자>/maps/lekiwi/map.yaml
```

---

## 빌드

```bash
cd ~/lekiwi_ws        # Pi 워크스페이스
cbs lekiwi_cartographer && sb
```

PC는 RViz 설정 파일(`src/lekiwi_cartographer/rviz/slam.rviz`)을 작업 사본에서 바로 연다. 로봇 모델을 보려고 `lekiwi_description`만 빌드한다.

**테스트** (cartographer_ros 없이 launch · Lua · RViz 계약과 `save_map.sh`를 가짜 `ros2`로 검사)

```bash
colcon test --packages-select lekiwi_cartographer --event-handlers console_direct+
colcon test-result --verbose
```

---

## 실행

1. **Pi 터미널 1**: 센서 · 베이스 bringup. `/lekiwi/sensor/scan_filtered`, `/lekiwi/sensor/offset_imu`, `/odom`, TF가 나와야 한다.
2. **Pi 터미널 2**: 지도 작성 시작. 시작 후 로봇을 잠시 정지해 IMU를 안정시킨다.

   ```bash
   ros2 launch lekiwi_cartographer cartographer.launch.py
   # IMU가 없는 장비 (bringup도 use_imu:=false)
   ros2 launch lekiwi_cartographer cartographer.launch.py configuration_basename:=lekiwi_2d.lua
   ```

3. **PC**: Pi와 같은 `ROS_DOMAIN_ID`(LeKiwi 기본 42) · `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`로 지도를 본다.

   ```bash
   rviz2 -d $LEKIWI_WS/src/lekiwi_cartographer/rviz/slam.rviz   # LEKIWI_WS: 저장소 README 1-②
   ```

4. **노트북 → Pi (`ssh -X`)**: `lekiwi_keyboard_teleop`으로 천천히 주행한다 (이동 경로 · 전원 차단 방법을 현장에서 먼저 확인).
5. **Pi 새 터미널**: 로봇을 정지하고 `ros2 run lekiwi_cartographer save_map.sh`
6. 저장을 확인한 뒤 키보드 창(Esc)과 Cartographer(Ctrl+C)를 종료한다. 바퀴 정지 · 토크 해제를 직접 확인한다.

---

## 확인 명령어

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic hz /lekiwi/sensor/scan_filtered` | 스캔이 약 10 Hz로 오는지 (PC에서 재면 Wi-Fi 경로까지 확인) |
| `ros2 topic hz /lekiwi/sensor/offset_imu` | IMU 설정일 때 약 100 Hz인지 |
| `ros2 run tf2_ros tf2_echo odom base_footprint` | 베이스 컨트롤러 odom TF |
| `ros2 run tf2_ros tf2_echo map odom` | Cartographer가 `map → odom`을 내는지 |
| `ros2 topic echo --once /map --field info` | 지도 크기(`width`, `height`)가 0보다 큰지 |
| `ros2 service list \| grep write_state` | 지도 저장 서비스가 있는지 |

---

## 처리 흐름

```text
(Pi) lekiwi_scan_filter ── /lekiwi/sensor/scan_filtered ──┐ scan
(Pi) lekiwi_imu ────────── /lekiwi/sensor/offset_imu ─────┤ imu (IMU 설정만)
(Pi) 베이스 컨트롤러 ────── /odom, TF odom → base_footprint ┤
(Pi) robot_state_publisher ─ TF base_footprint → … ───────┘
                                                          ▼
(Pi) cartographer_node ──── TF map → odom, 서브맵, /write_state
  └─ cartographer_occupancy_grid_node ── /map (0.05 m)
                                           ├─ save_map.sh (Pi) → ~/maps/lekiwi/map.yaml · map.pgm · map.pbstream
                                           └─ (PC) RViz slam.rviz로 보기
```

## 알려진 한계

- 주행 지도 작성은 좁은 공간(지나간 바닥 약 1.8 m²) 한 번만 확인했다. 넓은 방 · 되돌아와 이어 붙이기(loop closure)는 아직 확인하지 않았다.
- 지도 원점은 시작할 때의 `imu_link` 자세다 (`tracking_frame`). `imu_link`가 몸체 기준 +90° · 7.5 cm 위라서
  `map → odom`이 약 -90° · z -0.075 m로 나온다. RViz(고정 프레임 `map`)에서 지도가 로봇 시작 방향보다 90° 돌아 보이고
  로봇 바닥이 지도 면보다 7.5 cm 아래에 보인다. 위치 추정 · Nav2에는 영향이 없다 (저장소 설정도 같다).
- Pi CPU: 로봇을 세워 둔 상태에서 전체 약 30%(`cartographer_node`는 코어 하나의 약 36%). 주행 중 부하는 4단계에서 잰다.
- 스캔 `time_increment = 0`이라 이동 · 회전 중 스캔 왜곡을 보정하지 않는다. 천천히 주행한다.
- IMU 축 · 정지 bias · TF는 장비마다 확인한다. IMU 입력이 없는데 기본 `lekiwi_2d_imu.lua`를 쓰면 지도가 진행되지 않는다.
- 하부에 거꾸로 단 라이다는 세 바퀴 부근이 가려진다. 필터의 바퀴 구간 가림은 기본으로 꺼져 있다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `Package 'cartographer_ros' not found` | Pi에 Cartographer가 없음. [설치 (Pi, 한 번만)](#설치-pi-한-번만) 실행 |
| 지도가 생기지 않음 (`/map` 없음) | `/lekiwi/sensor/scan_filtered` · `/odom` · TF 수신 확인. IMU 설정이면 `/lekiwi/sensor/offset_imu`도 확인 |
| `Queue waiting for data: (0, imu)` 반복 | IMU가 안 옴. Pi IMU 확인 또는 `configuration_basename:=lekiwi_2d.lua` |
| TF 오류 (`lidar_link` · `imu_link`를 찾지 못함) | Pi의 `robot_state_publisher` 실행 확인 |
| RViz에 스캔이 안 보임 | Reliability Policy `Best Effort`, Pi 필터 실행 확인 |
| `ROS 환경이 없습니다` (save_map.sh) | `source /opt/ros/jazzy/setup.bash`와 워크스페이스 `install/setup.bash` 후 다시 실행 |
| `필요한 ROS 패키지가 없습니다: …` | Pi에 `nav2_map_server` · `cartographer_ros_msgs`가 없음. [설치](#설치-pi-한-번만) 실행 |
| `다른 지도 저장이 실행 중입니다` | 다른 터미널의 저장이 끝난 뒤 다시 실행 |
| `Cartographer 상태 저장 성공 응답을 확인하지 못했습니다` | Cartographer가 실행 중인지, 서비스 응답의 오류 메시지 확인. YAML · PGM은 남아 있다 |

---

## 패키지 구조

```text
lekiwi_cartographer/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── lekiwi_2d_imu.lua               # 기본 (IMU 사용)
│   └── lekiwi_2d.lua                   # IMU 없는 장비
├── launch/
│   ├── cartographer.launch.py          # SLAM + 지도 (+ RViz, 화면이 있을 때만)
│   └── occupancy_grid.launch.py        # 서브맵 → /map
├── rviz/slam.rviz
├── scripts/save_map.sh                 # 지도 저장 (lib/lekiwi_cartographer에 설치)
└── test/
    ├── test_cartographer_contract.py
    └── test_save_map.py
```

## 참고한 코드

- **LeKiwi 저장소 `lekiwi_cartographer`**: Lua 설정, launch, RViz 설정, 계약 테스트
- **LeKiwi 저장소 `tools/save_map.sh`, `tests/customer/test_save_map.py`**: 지도 저장 순서와 실패 처리, 회귀 테스트
- **LeKiwi 저장소 `docs/hardware/slam_navigation.md`**: 실행 순서, IMU 프로파일 선택, 검증 구분
- **설정 출처**: `yeon-03/lekiwi-pill-pickup` commit `9f4a00cfd9dd15e31e73f96825eb86193ef2a20a`. 조건은 워크스페이스 `THIRD_PARTY_NOTICES.md`

저장소와 다른 점:

- 스캔 필터를 띄우지 않는다 (`use_scan_filter` 인자 삭제). Pi가 `/lekiwi/sensor/scan_filtered`를 발행한다.
- 입력 기본값: `scan_topic` `/scan_navigation` → `lekiwi/sensor/scan_filtered`, `imu_topic` `/imu/data_raw` → `lekiwi/sensor/offset_imu`
- `save_map.sh`: 저장소 `tools/ros_env.sh` 대신 이미 source된 ROS 환경과 필요한 패키지를 확인한다. 저장 폴더는 `LEKIWI_MAP_DIR`, 없으면 `~/maps/lekiwi`.
  마지막 지도 기록은 저장소 `.colcon/last_map.txt` 대신 지도 폴더의 `last_map.txt`에 쓴다. 끝에 Nav2 실행 명령을 출력한다.

## 검증 기록

- 2026-10-18, PC (Ubuntu 24.04, Jazzy, cartographer_ros · Nav2 미설치): 빌드 성공. 테스트 30개 통과 (계약 7, `save_map.sh` 23)
- launch 설명 생성 확인 (`ros2 launch lekiwi_cartographer cartographer.launch.py --show-args`), Python 컴파일 · YAML 파싱 확인
- 2026-10-18, lekiwi04 (Pi 4, Jazzy, `cartographer_ros` 2.0.9003 apt), bringup과 함께, **로봇 정지 · 사람 없이**:
  - 빌드 성공, 테스트 32개 통과
  - 60초 실행: Lua를 읽고 IMU를 100 Hz로 받음(`imu rate: 100.00 Hz`), 서브맵 생성, `/map` 47 × 53칸(0.05 m), TF `map → odom` 발행
  - `LEKIWI_MAP_DIR=/tmp/...`로 `save_map.sh` 실행: `map.yaml` · `map.pgm` · `map.pbstream` 저장, 종료 코드 0
  - `Ctrl+C`로 종료하면 남는 프로세스 없음
- 2026-10-18 21:08 ~ 21:14, lekiwi04, **사람 있음**: bringup(관절 10 Hz) + Cartographer(Pi) + PC RViz(`slam.rviz`), `lekiwi_keyboard_teleop`으로 주행
  - `imu rate: 100.00 Hz`, `odom rate: 50.00 Hz`, `scan rate: 10.01 Hz` 수신. 로봇 정지 중 Pi CPU 전체 약 30%
  - `save_map.sh`: `~/maps/lekiwi/map.yaml` · `map.pgm`(120 × 104칸) · `map.pbstream`(2.4 MB), `code=0`
  - 지나간 바닥 약 1.8 m², 벽이 겹치지 않음. 이 지도로 Nav2 목표 도착 확인 (`lekiwi_navigation2` 검증 기록)
