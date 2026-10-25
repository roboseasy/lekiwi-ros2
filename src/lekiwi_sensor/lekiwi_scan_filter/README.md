# lekiwi_scan_filter

LeKiwi 라이다 스캔에서 **로봇 자신의 반사와 비정상 거리값을 NaN으로 지워** Cartographer · Nav2가 쓰는 스캔으로 다시 발행하는 노드 패키지.
LeKiwi 저장소 `lekiwi_sensors/scripts/navigation_scan_filter`(Python)를 같은 규칙으로 C++에 옮겼다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_sensor/lekiwi_scan_filter`
- 실행 위치: **Pi** (PC의 Cartographer · Nav2는 이 노드의 출력만 받는다)

> ✅ 라이다를 직접 열지 않는다. `lekiwi_lidar`가 발행한 토픽만 받으므로 YDLidar SDK가 없어도 빌드된다.
>
> ⚠️ 제외한 빔은 **NaN**(관측 없음)으로 바꾼다. `+inf`(빈 공간)로 바꾸지 않는다.
> 가려진 방향의 장애물을 지도 · costmap에서 지우지 않기 위해서다.

---

## 노드

- `lekiwi_scan_filter_node`

## 구독 (Subscribe)

| 토픽 | 타입 | QoS | 내용 |
|------|------|-----|------|
| `/lekiwi/sensor/scan` | `sensor_msgs/LaserScan` | `SensorDataQoS` | `lekiwi_lidar` 원본 스캔 (frame `lidar_link`) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | TF 기본 | **`mask_wheels: true`일 때만** 구독. `lidar_link → base_footprint` 회전 |

## 발행 (Publish)

| 토픽 | 타입 | 주기 | 내용 |
|------|------|------|------|
| `/lekiwi/sensor/scan_filtered` | `sensor_msgs/LaserScan` | 입력과 같음 (약 10 Hz) | 제외한 빔만 NaN으로 바꾼 스캔 |

- QoS: `SensorDataQoS` (best effort)
- 그대로 두는 것: `header`(시각 · frame), 배열 길이 · 순서, `angle_*`, `time_increment`, `scan_time`, `range_min` · `range_max`, `intensities`
- 걸러진 뒤 유한한 거리값이 하나도 없으면 **그 스캔은 내보내지 않는다** (`scan withheld` 오류를 2초에 한 번 남긴다).
  받는 쪽(Nav2 costmap · collision monitor)의 센서 timeout이 동작하게 하기 위해서다.

**빔을 제외하는 조건** (하나라도 맞으면 NaN)

| 조건 | 설명 |
|------|------|
| 유한값이 아님 | `NaN`, `±inf` |
| `< max(self_radius, range_min)` | 로봇 자기 반사. `lekiwi_lidar`가 빈 칸에 넣는 `0.0`도 여기서 걸러진다 |
| `> range_max` | 측정 범위 밖 |
| 세기 `< min_intensity` | `min_intensity > 0`일 때만. 세기 배열이 없으면 그 스캔은 내보내지 않는다 |
| 바퀴 구간 안 | `mask_wheels: true`일 때만. 빔 방향을 TF 회전(거꾸로 단 roll 포함)으로 `base_frame` 축에 옮겨 비교한다 |

## 파라미터

설정 파일: `config/lekiwi_scan_filter.yaml`. 값은 저장소 `lekiwi_sensors/param/navigation_scan.yaml`과 같고 토픽 이름만 바뀌었다.

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `input_topic` | string | `lekiwi/sensor/scan` | 입력 토픽 |
| `output_topic` | string | `lekiwi/sensor/scan_filtered` | 출력 토픽. 입력과 같으면 시작하지 않는다 |
| `base_frame` | string | `base_footprint` | 바퀴 구간의 기준 축 |
| `self_radius` | double | `0.19` | [m] 라이다 원점 기준 자기 반사 거리. 유한한 0 이상 |
| `min_intensity` | double | `0.0` | 세기 하한. 0이면 검사하지 않는다. 유한한 0 이상 |
| `mask_wheels` | bool | `false` | 바퀴 구간 가림. 켜면 TF가 필요하다 |
| `wheel_sectors` | string[] | `['179.5:38.5', '61.0:35.5', '-59.3:34.0']` | `"중심:폭"` [deg]. `0 < 폭 < 360`. `mask_wheels: true`일 때만 해석한다 |

- `self_radius`는 Nav2의 AMCL `laser_min_range`, costmap `obstacle_min_range` · `raytrace_min_range`와 같은 값으로 둔다 (`lekiwi_navigation2` 테스트가 확인한다).
- 바퀴 구간은 참고 로봇에서 본 방향이다. 구간 전체를 지우면 먼 반사도 사라지므로 기본으로 켜지 않는다.

---

## 빌드

```bash
cd ~/lekiwi_ws
cbs lekiwi_scan_filter && sb
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나 `cb` / `cbs` / `cbsource`를 사용한다.

`config/` · `launch/` 파일은 `--symlink-install`로 연결되므로 수정 후 노드만 재시작하면 적용된다. `.cpp` · `.hpp`를 고쳤다면 다시 빌드한다.

**테스트** (필터 함수 gtest, ROS 통신 없음)

```bash
colcon test --packages-select lekiwi_scan_filter --event-handlers console_direct+
colcon test-result --verbose
```

---

## 실행

라이다 노드와 함께 실행한다. 보통은 `lekiwi_bringup`이 함께 띄운다.

```bash
ros2 launch lekiwi_lidar lekiwi_lidar.launch.py          # 터미널 1
ros2 launch lekiwi_scan_filter lekiwi_scan_filter.launch.py   # 터미널 2
```

정상이면 이런 로그가 나온다. `< >` 부분은 환경에 따라 다르다.

```text
scan filter: /lekiwi/sensor/scan -> /lekiwi/sensor/scan_filtered, self_radius 0.190 m, min_intensity 0.0, wheel mask off
first scan: <배열 크기> ranges, <입력 유효 개수> valid in, <남은 개수> kept (frame lidar_link)
```

종료는 `Ctrl+C`. 이 launch는 노드가 끝나면 launch 전체를 끝낸다.

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_scan_filter/config/lekiwi_scan_filter.yaml` | 파라미터 파일 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |

**바퀴 구간 가림을 이번 실행에서만 켜기** (TF `base_footprint → lidar_link`가 있어야 한다)

```bash
ros2 run lekiwi_scan_filter lekiwi_scan_filter_node --ros-args \
  --params-file ~/lekiwi_ws/src/lekiwi_sensor/lekiwi_scan_filter/config/lekiwi_scan_filter.yaml \
  -p mask_wheels:=true
```

---

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic hz /lekiwi/sensor/scan_filtered` | 입력과 같은 약 10 Hz인지 |
| `ros2 topic echo --once /lekiwi/sensor/scan_filtered --no-arr` | header · 각도 · 시간이 입력과 같은지 |
| `ros2 topic echo --once /lekiwi/sensor/scan_filtered --field ranges` | 가까운 값이 `nan`으로 바뀌었는지 |
| `ros2 param get /lekiwi_scan_filter_node self_radius` | 현재 값 확인 |

**RViz**: Fixed Frame `base_link`, LaserScan 두 개(`/lekiwi/sensor/scan`, `/lekiwi/sensor/scan_filtered`, Reliability `Best Effort`)를 다른 색으로 겹쳐 보면
로봇 몸체 · 바퀴 근처 점만 사라졌는지 볼 수 있다.

---

## 처리 흐름

```text
/lekiwi/sensor/scan (lekiwi_lidar, frame lidar_link)
  └─ 빔마다 (배열 순서 그대로)
      ├─ 유한값 아님 · < max(self_radius, range_min) · > range_max → NaN
      ├─ (min_intensity > 0) 세기 부족 → NaN
      └─ (mask_wheels) TF 회전으로 base_frame 방위각 → 바퀴 구간 안 → NaN
  ├─ 남은 유한값 없음 → 내보내지 않음 (오류 로그, 2초 간격)
  └─ LaserScan ──────────────────────────────→ /lekiwi/sensor/scan_filtered
```

## 알려진 한계

- `self_radius` 0.19 m는 시작값이다. 조립한 장비마다 바퀴 반사 거리와, 가까운 실제 장애물이 지워지지 않는지를 확인한다.
- `mask_wheels`는 TF를 스캔 시각으로 조회하고 기다리지 않는다. `lidar_link`까지 고정 관절이면 `/tf_static`만으로 된다. TF가 없으면 스캔을 내보내지 않는다.
- 바퀴 구간 각도는 참고 로봇 값이다. 현재 장착 기준으로 다시 측정하기 전에는 확정값으로 쓰지 않는다.
- 이동 중 스캔 왜곡(`time_increment = 0`)은 보정하지 않는다. 필터는 시간 정보를 바꾸지 않는다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `input and output topics must differ` | 두 토픽이 같은 이름으로 풀림. `output_topic`을 바꾼다 |
| `radius and intensity threshold must be finite and nonnegative` | `self_radius` · `min_intensity`가 음수이거나 유한값이 아님 |
| `wheel sector must be finite center:width degrees ...` | `wheel_sectors` 형식 오류. `"중심:폭"`, `0 < 폭 < 360` |
| `scan withheld: no usable ranges after filtering` | 모든 빔이 제외됨. 라이다가 막혔는지, `self_radius`가 너무 큰지 확인 |
| `scan withheld: "base_footprint" passed to lookupTransform ... does not exist` | `mask_wheels: true`인데 TF가 없음. `robot_state_publisher` 실행 확인 |
| `scan withheld: intensity filtering requested but intensities are missing` | `min_intensity > 0`인데 세기 배열이 없음. 라이다 `intensity: true` 확인 |
| `/lekiwi/sensor/scan_filtered`가 안 나옴 | 입력이 오는지 `ros2 topic hz /lekiwi/sensor/scan`, 노드 로그의 `scan withheld` 확인 |

---

## 패키지 구조

```text
lekiwi_scan_filter/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_scan_filter.yaml        # 파라미터
├── launch/lekiwi_scan_filter.launch.py   # 실행 파일
├── include/lekiwi_scan_filter/
│   ├── scan_filter.hpp                   # 필터 함수 (ROS 통신 없음)
│   └── lekiwi_scan_filter_node.hpp       # ROS 2 노드
├── src/
│   ├── scan_filter.cpp
│   └── lekiwi_scan_filter_node.cpp       # main 포함
└── test/test_scan_filter.cpp             # gtest
```

## 참고한 코드

- **LeKiwi 저장소 `lekiwi_sensors/scripts/navigation_scan_filter`**: 제외 조건, NaN 처리, 회전 공식, 입력 검증, 출력 보류 규칙
- **LeKiwi 저장소 `lekiwi_sensors/param/navigation_scan.yaml`**: 기본값과 바퀴 구간
- **LeKiwi 저장소 `lekiwi_sensors/test/test_navigation_scan_filter.py`**: gtest로 옮긴 경우 (역장착 · ±π 경계 · 세기 · 잘못된 설정)
- **LeKiwi 저장소 `docs/hardware/slam_navigation.md`, `docs/hardware/lidar_slam_followup.md`**: 0.19 m 기준, 가림 구간을 기본으로 켜지 않는 이유
- **lekiwi_lidar**: 패키지 구조, 파라미터 선언 · 검증, 로그 형식

원본과 다른 점: TF 구독은 `mask_wheels: true`일 때만 만든다. 길이가 1이 아닌 회전 quaternion은 정규화하고, 길이 0이면 그 스캔을 내보내지 않는다.
`mask_wheels: true`이면 빈 `base_frame`을 시작할 때 거부한다.

## 검증 기록

- 2026-10-18, PC (Ubuntu 24.04, Jazzy): 빌드 성공, 빌드 경고 0. gtest 12개 통과
- 2026-10-18, PC 독립 domain(77, LOCALHOST)에서 합성 LaserScan으로 실행:
  - `lekiwi_scan_filter.launch.py` 기본 설정: 입력 `[0.0, 0.1, 0.18, 0.5, inf, nan, 13.0, 2.0]` → 출력 `[nan, nan, nan, 0.5, nan, nan, nan, 2.0]`. header · 각도 · 시간 · 세기 유지
  - 모든 빔이 0.1 m인 스캔은 내보내지 않음 (`scan withheld` 1회 로그), 다음 정상 스캔에서 다시 발행
  - `mask_wheels:=true`, TF 없음: 내보내지 않음. 정적 TF `base_footprint → lidar_link` roll π: 360개 중 107개 NaN, 따로 계산한 값과 일치
  - 입력 = 출력 토픽, 음수 `self_radius`, `'0:360'` 구간: 시작 시 오류로 종료 (exit 1)
- Pi 빌드, 실제 라이다 스캔에서의 동작, RViz 확인은 아직 하지 않았다.
