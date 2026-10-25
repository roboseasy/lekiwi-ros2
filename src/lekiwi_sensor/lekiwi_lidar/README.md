# lekiwi_lidar

LeKiwi Raspberry Pi의 USB에 연결된 **YDLIDAR Tmini Plus를 YDLidar SDK로 직접 읽어 `sensor_msgs/LaserScan`으로 발행**하는 노드 패키지.
라이다 설정값은 LeKiwi `lekiwi_sensors/param/ydlidar.yaml`(2026-09-22 정지 방향 확인)과 같다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · YDLIDAR Tmini Plus
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_sensor/lekiwi_lidar`

> ⚠️ 같은 라이다를 쓰는 `lekiwi_sensors/ydlidar.launch.py`(LeKiwi 브링업 `use_lidar:=true`)나 `ydlidar_ros2_driver`와 **동시에 실행하지 않는다.**
> 시리얼 포트 하나를 두 프로그램이 함께 쓸 수 없다.

---

## 하드웨어 연결

Tmini Plus → USB 어댑터 보드(CP2102 USB-UART) → Raspberry Pi USB 포트

연결이 맞으면 CP2102 장치가 `ttyUSB0`으로 보인다.

```bash
ls -l /dev/serial/by-id/
# ... usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

> `usb-1a86_...` 장치(`ttyACM0`)는 라이다가 아니다. 다른 USB 시리얼 장치를 먼저 꽂아 번호가 바뀌면 `port:=` 인자로 지정한다.

## YDLidar SDK

빌드하려면 YDLidar SDK가 `/usr/local`에 설치되어 있어야 한다. lekiwi04에는 설치되어 있다.

```bash
ls /usr/local/lib/cmake/ydlidar_sdk     # ydlidar_sdkConfig.cmake가 보이면 설치됨
```

없으면 SDK 소스(`~/ydlidar_ros2_ws/src/YDLidar-SDK`)에서 설치한다.

```bash
cd ~/ydlidar_ros2_ws/src/YDLidar-SDK
mkdir -p build && cd build
cmake .. && make -j4 && sudo make install
```

---

## 노드

- `lekiwi_lidar_node`

## 구독 (Subscribe)

- 없음 — YDLidar SDK로 시리얼 포트를 직접 읽는다.

## 발행 (Publish)

| 토픽 | 타입 | 주기 | 내용 |
|------|------|------|------|
| `/lekiwi/sensor/scan` | `sensor_msgs/LaserScan` | 라이다 한 바퀴마다 (약 10 Hz) | 360° 거리 · 세기 배열 |

- `frame_id`: `lidar_link` / 단위: 거리 m, 각도 rad
- QoS: `SensorDataQoS` (best effort)
- `header.stamp`: 한 바퀴의 첫 측정점 시각 (SDK가 시스템 시계로 기록)
- `time_increment`: 기본 `0` (`zero_time_increment: true`, LeKiwi `/scan`과 같음)
- 측정점이 없는 칸: `0.0` (`range_min`보다 작아 무효로 취급). `invalid_range_is_inf: true`면 `+inf`
- TF는 발행하지 않는다. `lidar_link` 위치는 LeKiwi 브링업의 `robot_state_publisher`가 준다.

## 파라미터

설정 파일: `config/lekiwi_lidar.yaml`. 이름은 `lekiwi_sensors/param/ydlidar.yaml`과 같다.

**연결**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | 시리얼 포트. launch 인자 `port`가 덮어쓴다 |
| `baudrate` | int | `230400` | 통신 속도 |
| `frame_id` | string | `lidar_link` | 메시지 frame |
| `auto_reconnect` | bool | `true` | 통신이 끊기면 SDK가 다시 연결을 시도 |

**Tmini Plus 고정값** (바꾸지 않는다)

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `lidar_type` | int | `1` | `TYPE_TRIANGLE` |
| `device_type` | int | `0` | `YDLIDAR_TYPE_SERIAL` |
| `isSingleChannel` | bool | `false` | 단방향 통신 여부 |
| `support_motor_dtr` | bool | `false` | DTR로 모터 제어 여부 |
| `sample_rate` | int | `4` | [K 샘플/s] |
| `intensity` / `intensity_bit` | bool / int | `true` / `8` | 세기 값 사용 / 비트 수 |
| `abnormal_check_count` | int | `4` | 시작할 때 정상 데이터를 확인하는 최대 시도 횟수 |

**스캔 범위 · 방향**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `frequency` | double | `10.0` | [Hz] 회전 속도 |
| `angle_min` / `angle_max` | double | `-180.0` / `180.0` | [deg] 스캔 각도 범위 |
| `range_min` / `range_max` | double | `0.05` / `12.0` | [m] 거리 범위 |
| `fixed_resolution` | bool | `true` | 회전마다 같은 개수의 배열로 낸다 |
| `reversion` | bool | `false` | 스캔을 180° 회전 |
| `inverted` | bool | `true` | 스캔 각도 부호 반전 |

**출력**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `scan_topic` | string | `lekiwi/sensor/scan` | 발행 토픽. launch 인자 `scan_topic`이 덮어쓴다 |
| `invalid_range_is_inf` | bool | `false` | 측정점이 없는 칸을 `0.0` 대신 `+inf`로 |
| `zero_time_increment` | bool | `true` | `time_increment`를 0으로. 검증 전에는 끄지 않는다 ([알려진 한계](#알려진-한계)) |

---

## 빌드

```bash
cd ~/lekiwi_ws
colcon build --symlink-install --packages-select lekiwi_lidar
source install/setup.bash
```

RobosEasy `~/.bashrc` alias를 설정했다면:

```bash
cbs lekiwi_lidar && sb      # 빌드 후 소스
cbsource lekiwi_lidar       # 위와 같음
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나, venv를 자동으로 빼고 빌드하는 `cb` / `cbs` / `cbsource`를 사용한다.

`config/` · `launch/` 파일은 `--symlink-install`로 설치 폴더에 연결되므로 **수정 후 다시 빌드하지 않아도** 노드를 재시작하면 적용된다. `.cpp` · `.hpp`를 고쳤다면 다시 빌드한다.

---

## 실행

```bash
ros2 launch lekiwi_lidar lekiwi_lidar.launch.py
```

라이다가 돌기 시작하고, SDK가 출력하는 `[YDLIDAR]` 줄 다음에 이런 로그가 나온다. `< >` 부분은 환경에 따라 다르다.

```text
YDLIDAR ready on /dev/ttyUSB0: 230400 baud, 10.0 Hz, angle -180~180 deg, range 0.05~12.0 m, frame=lidar_link
publishing /lekiwi/sensor/scan
first scan: <배열 크기> ranges (<유효 개수> valid, nearest <가장 가까운 거리> m), <회전 속도> Hz
```

종료는 `Ctrl+C`. 노드가 끝날 때 라이다 회전을 멈추고 포트를 닫는다.

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_lidar/config/lekiwi_lidar.yaml` | 파라미터 파일 |
| `port` | `/dev/ttyUSB0` | 시리얼 포트 |
| `scan_topic` | `lekiwi/sensor/scan` | 발행 토픽. SLAM 입력으로 쓰려면 `/scan` |

```bash
# 예: 고정 경로로 포트 지정
ros2 launch lekiwi_lidar lekiwi_lidar.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

**launch 인자에 없는 파라미터를 이번 실행에서만 바꾸기** (`-p`가 파일 값을 덮어쓴다)

```bash
ros2 run lekiwi_lidar lekiwi_lidar_node --ros-args \
  --params-file ~/lekiwi_ws/src/lekiwi_sensor/lekiwi_lidar/config/lekiwi_lidar.yaml \
  -p invalid_range_is_inf:=true
```

계속 바꿔 쓰려면 `config/lekiwi_lidar.yaml`을 편집하고 노드를 재시작한다.

---

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다. 괄호는 RobosEasy alias.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic list` (`rt`) | `/lekiwi/sensor/scan`이 보이는지 |
| `ros2 node list` (`rn`) | `/lekiwi_lidar_node`가 보이는지 |
| `ros2 topic hz /lekiwi/sensor/scan` | 약 10 Hz인지 |
| `ros2 topic echo --once /lekiwi/sensor/scan --no-arr` | 배열을 빼고 header · 각도 · 거리 범위만 보기 |
| `ros2 topic echo --once /lekiwi/sensor/scan --field ranges` | 거리 배열 한 번 보기 |
| `ros2 param get /lekiwi_lidar_node inverted` | 현재 값 확인 |

**PC에서 Pi 토픽 보기**: PC와 Pi의 도메인 · 통신 범위가 같아야 한다 (LeKiwi 기본 42 / SUBNET).

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic list
```

---

## RViz에서 스캔 보기

1. Pi: `ros2 launch lekiwi_lidar lekiwi_lidar.launch.py`
2. PC: 위 환경 변수를 맞춘 터미널에서 `rviz2` 실행
3. 왼쪽 **Global Options → Fixed Frame**에 `lidar_link`를 직접 입력
4. **Add → By topic → `/lekiwi/sensor/scan` → LaserScan** 추가
5. 점이 안 보이면 LaserScan의 **Topic → Reliability Policy**를 `Best Effort`로 바꾼다
6. 점이 작으면 **Size (m)**를 `0.03` 정도로 키운다

> ⚠️ LeKiwi는 라이다를 거꾸로 달았으므로 `lidar_link`도 뒤집혀 있다 (roll 180°).
> Fixed Frame을 `lidar_link`로 두면 로봇 **아래에서 올려다본 모습**이라 위에서 본 모습과 좌우가 반대다.
> 실제 앞뒤 · 좌우 확인은 브링업의 `robot_state_publisher`가 TF를 줄 때 Fixed Frame `base_link`로 한다.

---

## 처리 흐름

```text
Tmini Plus (USB 시리얼 230400, 약 10 Hz 회전)
  └─ YDLidar SDK (CYdLidar): reversion · inverted 적용, 한 바퀴 측정점 목록
      └─ 각도 칸 배치 (ydlidar_ros2_driver와 같은 규칙, angle_min부터 각도 순)
          ├─ 측정점이 없는 칸: 0.0 (또는 +inf)
          ├─ time_increment: 0 (zero_time_increment)
          └─ LaserScan ─────────────────────────────→ /lekiwi/sensor/scan
```

## 알려진 한계

- 하부에 거꾸로 장착해 **세 바퀴 부근이 가려진다.** 자체 가림 필터는 없다 (LeKiwi `docs/hardware/lidar_slam_followup.md`).
- `time_increment = 0`이라 이동 · 회전 중 스캔 왜곡을 보정하지 않는다. `inverted` 뒤 배열 순서와 실제 측정 순서가 맞는지 확인하기 전에는 `zero_time_increment`를 끄지 않는다.
- `reversion: false` · `inverted: true`는 정지 상태에서 방향을 확인한 조합이다. 장비별 현장 검증이 필요하다.
- LeKiwi 브링업과 Cartographer는 `lekiwi_sensors`의 `/scan`을 사용한다. 이 패키지는 브링업에 연결되어 있지 않다.
  SLAM 입력으로 바꿔 쓰려면 브링업을 `use_lidar:=false`로 실행하고 이 노드를 `scan_topic:=/scan`으로 실행한다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `ros2: command not found` | ROS를 source하지 않음. `source /opt/ros/jazzy/setup.bash` 또는 RobosEasy `~/.bashrc` 설정 |
| `Package 'lekiwi_lidar' not found` | 워크스페이스를 source하지 않음. `source ~/lekiwi_ws/install/setup.bash` (`sb`) |
| 빌드 중 `No module named 'catkin_pkg'` | lerobot venv가 켜져 있음. `deactivate` 후 빌드하거나 `cbs` 사용 |
| 빌드 중 `Could not find a package configuration file provided by "ydlidar_sdk"` | SDK가 설치되지 않음. [YDLidar SDK](#ydlidar-sdk) 참고 |
| `failed to initialize YDLIDAR on /dev/ttyUSB0` | USB 연결 · 포트 번호 확인 (`ls -l /dev/serial/by-id/`). 다른 프로그램이 포트를 쓰는지 `fuser -v /dev/ttyUSB0` |
| `Permission denied` | 사용자에게 `dialout` 그룹이 필요. `id`로 확인 후 그룹 추가, 재로그인 |
| `YDLIDAR scan failed` 반복 | 케이블 접촉 불량 가능. `auto_reconnect`가 다시 연결을 시도하고, 회복되면 `recovered` 로그가 나온다 |
| PC에서 토픽이 안 보임 | PC · Pi의 `ROS_DOMAIN_ID`(42) · `ROS_AUTOMATIC_DISCOVERY_RANGE`(SUBNET)가 같은지, 네트워크 연결 확인 |
| RViz에 점이 안 보임 | Fixed Frame이 `lidar_link`인지, Reliability Policy가 `Best Effort`인지 확인 |

---

## 패키지 구조

```text
lekiwi_lidar/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_lidar.yaml          # 파라미터
├── launch/lekiwi_lidar.launch.py     # 실행 파일
├── include/lekiwi_lidar/
│   ├── ydlidar_driver.hpp            # YDLidar SDK 드라이버
│   └── lekiwi_lidar_node.hpp         # ROS 2 노드
└── src/
    ├── ydlidar_driver.cpp
    └── lekiwi_lidar_node.cpp          # main 포함
```

## 참고한 코드

- **LeKiwi `lekiwi_sensors`**: `param/ydlidar.yaml`의 Tmini Plus 설정과 방향 조합, `scripts/scan_timing_normalizer`의 `time_increment = 0` 처리
- **ydlidar_ros2_driver** (`~/ydlidar_ros2_ws/src/ydlidar_ros2_driver`): SDK 옵션 설정, SDK 스캔 → `LaserScan` 각도 칸 배치 규칙
- **LeKiwi `lekiwi-sensor-check/native/lidar_probe.cpp`**: 초기화 실패 시에도 포트를 닫는 종료 처리
- **lekiwi_imu**: 패키지 구조, 파라미터 선언 · 검증, 로그 형식
- **LeKiwi `docs/hardware/lidar_slam_followup.md`**: 장착 · 가림 · 시간 처리 한계

## 검증 기록

- 2026-10-17, lekiwi04 (Pi 4, Jazzy): 임시 폴더에서 빌드 성공, 빌드 경고 0
- 실제 라이다로 실행, `/lekiwi/sensor/scan` 주기, RViz 방향은 아직 검증하지 않았다.
