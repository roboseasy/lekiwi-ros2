# lekiwi_imu

LeKiwi Raspberry Pi의 I2C에 연결된 **BMI160 IMU를 직접 읽어 `sensor_msgs/Imu`로 발행**하는 노드 패키지.
시작 시 자이로 바이어스 보정, One Euro · 칼만 필터, 자세(roll · pitch · yaw) 추정을 포함한다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · GY-BMI160 모듈
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_sensor/lekiwi_imu`

> ⚠️ 같은 BMI160을 쓰는 `lekiwi_sensors/bmi160_node`(LeKiwi 브링업 `use_imu:=true`)와 **동시에 실행하지 않는다.**
> 이 노드는 시작할 때 센서를 리셋하므로 다른 노드의 측정이 깨진다.

---

## 하드웨어 연결

GY-BMI160 → Raspberry Pi 4 GPIO (USB·랜 포트를 아래로 두었을 때 왼쪽 줄 = 홀수, 오른쪽 줄 = 짝수)

| 모듈 핀 | Pi 핀 | Pi 핀 이름 |
|---------|:-----:|-----------|
| VIN | 1 | 3V3 power |
| GND | 6 | Ground |
| SCL | 5 | GPIO 3 (SCL) |
| SDA | 3 | GPIO 2 (SDA) |
| SA0 | 9 | Ground → 주소 `0x68` |
| 3.3V, CS, OCS, INT1, INT2, SCX, SDX | – | 연결 안 함 |

> ⚠️ 2번 · 4번 핀은 **5V**다. VIN을 연결하지 않는다. 선은 전원을 끈 상태에서 꽂는다.

연결이 맞으면 I2C 버스 1의 `0x68`에서 칩 ID `0xD1`이 읽힌다.

---

## 노드

- `lekiwi_imu_node`

## 구독 (Subscribe)

- 없음 — `/dev/i2c-1`에서 BMI160 레지스터를 직접 읽는다.

## 발행 (Publish)

| 토픽 | 타입 | 발행 조건 | 내용 |
|------|------|----------|------|
| `/lekiwi/sensor/offset_imu` | `sensor_msgs/Imu` | 항상 | 자이로 바이어스만 제거한 값. 자세 없음 (`orientation_covariance[0] = -1`) |
| `/lekiwi/sensor/lpf_imu` | `sensor_msgs/Imu` | `use_low_pass_filter: true` (기본 켜짐) | One Euro 필터 값 + 자세 |
| `/lekiwi/sensor/calibrated_imu` | `sensor_msgs/Imu` | `use_kalman_filter: true` (기본 꺼짐) | 칼만 필터 값 + 자세 |
| `/tf` (`imu_world` → `imu_orientation`) | `tf2_msgs/TFMessage` | `publish_orientation_tf: true` (기본 꺼짐) | RViz 확인용 자세 |

- `frame_id`: `imu_link` / 단위: 가속도 m/s², 각속도 rad/s, 자세 quaternion
- QoS: `SensorDataQoS` (best effort)
- 주기: `sample_rate_hz` (기본 100 Hz). 시작 후 바이어스 보정(약 2초)이 끝나야 발행을 시작한다.

## 파라미터

설정 파일: `config/lekiwi_imu.yaml`

**BMI160 연결**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `i2c_device` | string | `/dev/i2c-1` | I2C 장치 |
| `i2c_address` | int | `104` | 7-bit 주소. 104 = `0x68`(SA0=GND), 105 = `0x69` |
| `sample_rate_hz` | int | `100` | 25, 50, 100, 200, 400 |
| `accel_range_g` | int | `2` | 2, 4, 8, 16 |
| `gyro_range_dps` | int | `250` | 125, 250, 500, 1000, 2000 |
| `frame_id` | string | `imu_link` | 메시지 frame |
| `axis_map` | int[3] | `[1, 2, 3]` | 센서 축 → `imu_link` 축. 부호 있는 순열 (예: `[-2, 1, 3]`) |

**보정 · 장애 처리**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `gyro_bias_samples` | int | `200` | 시작 시 바이어스 수집 샘플 수 (100 Hz에서 2초). 0이면 보정 안 함 |
| `calibration_max_gyro_std` | double | `0.02` | [rad/s] 수집 중 표준편차가 넘으면 움직였다고 보고 다시 수집 |
| `reinit_after_failures` | int | `100` | 연속 읽기 실패가 이 횟수만큼 쌓일 때마다 센서 재초기화 |

**토픽 · 공분산**

| 이름 | 타입 | 기본값 |
|------|------|--------|
| `offset_topic` | string | `lekiwi/sensor/offset_imu` |
| `lpf_topic` | string | `lekiwi/sensor/lpf_imu` |
| `kf_topic` | string | `lekiwi/sensor/calibrated_imu` |
| `angular_velocity_variance` | double | `0.0004` |
| `linear_acceleration_variance` | double | `0.04` |
| `orientation_variance_roll_pitch` | double | `0.0025` |
| `orientation_variance_yaw` | double | `0.05` |

**필터** (One Euro 단위: 가속도 g, 각속도 deg/s)

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `use_low_pass_filter` | bool | `true` | One Euro 필터 + `lpf_imu` 발행 |
| `use_kalman_filter` | bool | `false` | 칼만 필터 + `calibrated_imu` 발행 |
| `accel_min_cutoff` / `accel_beta` | double | `1.0` / `0.1` | 가속도 최소 차단 주파수 [Hz] / 지연 감소 계수 |
| `gyro_min_cutoff` / `gyro_beta` | double | `2.0` / `0.02` | 각속도 최소 차단 주파수 [Hz] / 지연 감소 계수 |
| `dcutoff` | double | `1.0` | 미분값 필터 차단 주파수 [Hz] |
| `Q_cov` / `R_cov` | double | `0.000005` / `0.0001` | 칼만 프로세스 / 측정 노이즈 |

**자세 추정 · TF**

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `zero_vel_accel_threshold` | double | `0.05` | [g] 중력을 뺀 가속도가 이보다 작으면 정지 후보 |
| `zero_vel_gyro_threshold` | double | `1.0` | [deg/s] 각속도가 이보다 작으면 정지 후보 |
| `gyro_deadband` | double | `0.01` | [rad/s] 이보다 느린 회전은 적분하지 않음 |
| `publish_orientation_tf` | bool | `false` | RViz 확인용 자세 TF 발행 |
| `orientation_tf_parent` / `orientation_tf_child` | string | `imu_world` / `imu_orientation` | 로봇 TF 트리(`imu_link`)와 겹치지 않는 별도 프레임 |

---

## 빌드

```bash
cd ~/lekiwi_ws
colcon build --symlink-install --packages-select lekiwi_imu
source install/setup.bash
```

RobosEasy `~/.bashrc` alias를 설정했다면:

```bash
cbs lekiwi_imu && sb        # 빌드 후 소스
cbsource lekiwi_imu         # 위와 같음
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나, venv를 자동으로 빼고 빌드하는 `cb` / `cbs` / `cbsource`를 사용한다.

의존성이 없으면 설치한다.

```bash
cd ~/lekiwi_ws
rosdep install --from-paths src/lekiwi_sensor/lekiwi_imu --ignore-src --rosdistro jazzy -y
```

`config/` · `launch/` 파일은 `--symlink-install`로 설치 폴더에 연결되므로 **수정 후 다시 빌드하지 않아도** 노드를 재시작하면 적용된다. `.cpp` · `.hpp`를 고쳤다면 다시 빌드한다.

---

## 실행

```bash
ros2 launch lekiwi_imu lekiwi_imu.launch.py
```

**시작 후 2초 동안 로봇을 움직이지 않는다** (자이로 바이어스 보정). 정상이면 다음 로그가 나온다.

```text
BMI160 ready on /dev/i2c-1 address 0x68: 100 Hz, +/-2 g, +/-250 dps, frame=imu_link
keep the robot still: collecting 200 gyro bias samples
gyro bias ready: [-0.00758, -0.00198, -0.00761] rad/s (std 0.00094)
publishing /lekiwi/sensor/offset_imu, /lekiwi/sensor/lpf_imu
initial orientation from gravity: roll=-2.04 deg, pitch=4.86 deg
```

종료는 `Ctrl+C`.

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_imu/config/lekiwi_imu.yaml` | 파라미터 파일 |
| `i2c_address` | `104` | `0x69` 모듈이면 `105` |
| `publish_orientation_tf` | `false` | RViz 확인용 자세 TF 발행 |

```bash
# 예: 0x69 주소 모듈 + RViz용 TF 켜기
ros2 launch lekiwi_imu lekiwi_imu.launch.py i2c_address:=105 publish_orientation_tf:=true
```

**launch 인자에 없는 파라미터를 이번 실행에서만 바꾸기** (`-p`가 파일 값을 덮어쓴다)

```bash
ros2 run lekiwi_imu lekiwi_imu_node --ros-args \
  --params-file ~/lekiwi_ws/src/lekiwi_sensor/lekiwi_imu/config/lekiwi_imu.yaml \
  -p use_kalman_filter:=true
```

계속 바꿔 쓰려면 `config/lekiwi_imu.yaml`을 편집하고 노드를 재시작한다.

---

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다. 괄호는 RobosEasy alias.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic list` (`rt`) | `/lekiwi/sensor/...` 토픽이 보이는지 |
| `ros2 node list` (`rn`) | `/lekiwi_imu_node`가 보이는지 |
| `ros2 topic hz /lekiwi/sensor/lpf_imu` | 약 100 Hz인지 |
| `ros2 topic echo /lekiwi/sensor/lpf_imu --field orientation` (`re ...`) | 로봇을 기울이면 x · y가 바뀌는지 |
| `ros2 topic echo --once /lekiwi/sensor/offset_imu` | 값 한 번 보기 |
| `ros2 param list /lekiwi_imu_node` | 파라미터 목록 |
| `ros2 param get /lekiwi_imu_node use_kalman_filter` | 현재 값 확인 |

**정지 상태의 정상 값**: `linear_acceleration.z` ≈ +9.8, `angular_velocity` 세 축 ≈ 0, `orientation`의 x · y는 기울어진 만큼만 작게 나온다.

**PC에서 Pi 토픽 보기**: PC와 Pi의 도메인 · 통신 범위가 같아야 한다 (LeKiwi 기본 42 / SUBNET).

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic list
```

---

## RViz에서 자세 보기

1. Pi: `ros2 launch lekiwi_imu lekiwi_imu.launch.py publish_orientation_tf:=true`
2. PC: 위 환경 변수를 맞춘 터미널에서 `rviz2` 실행
3. 왼쪽 **Global Options → Fixed Frame**에 `imu_world`를 직접 입력
4. **Add → TF** 추가 → 로봇을 기울이거나 돌리면 `imu_orientation` 축이 함께 움직인다

---

## 처리 흐름

```text
BMI160 (I2C, 100 Hz)
  └─ axis_map (센서 축 → imu_link 축)
      └─ 자이로 바이어스 제거 ─────────────────────────→ offset_imu
          ├─ One Euro 필터 ([g], [deg/s]) ────────────→ lpf_imu
          ├─ 칼만 필터 (F = H = I, 6상태) ─────────────→ calibrated_imu
          └─ 자세 추정 (입력: 칼만 > One Euro > offset 순서)
               1) 각속도 적분 → 증분 quaternion
               2) 가속도를 world로 회전 후 중력 제거
               3) 정지 판별 시: yaw 고정 + roll/pitch를 중력 방향으로 재보정
```

## 알려진 한계

- **yaw는 자이로 적분만으로 구하므로 시간이 지나면 조금씩 틀어진다.** 지자기 센서가 없어 절대 방향(북쪽 기준)은 알 수 없다.
- `axis_map` 기본값 `[1, 2, 3]`과 `imu_link` 장착 방향은 장비별 현장 검증이 필요하다.
- 필터 파라미터는 LeKiwi 100 Hz용 시작값이며 튜닝이 필요하다.
- LeKiwi 브링업과 Cartographer는 `lekiwi_sensors`의 `/imu/data_raw`를 사용한다. 이 패키지는 브링업에 연결되어 있지 않다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `ros2: command not found` | ROS를 source하지 않음. `source /opt/ros/jazzy/setup.bash` 또는 RobosEasy `~/.bashrc` 설정 |
| `Package 'lekiwi_imu' not found` | 워크스페이스를 source하지 않음. `source ~/lekiwi_ws/install/setup.bash` (`sb`) |
| 빌드 중 `No module named 'catkin_pkg'` | lerobot venv가 켜져 있음. `deactivate` 후 빌드하거나 `cbs` 사용 |
| `BMI160 not found ... (no response)` | 배선 · 전원 확인. 위 연결표대로 꽂혔는지, 한 칸 밀리지 않았는지 확인 |
| `BMI160 not found ... (chip id 0x.., expected 0xD1)` | 다른 장치 또는 다른 주소. SA0 배선 확인, `0x69`면 `i2c_address:=105` |
| `failed to open /dev/i2c-1: Permission denied` | 사용자에게 `i2c` 또는 `dialout` 그룹이 필요. `id`로 확인 후 그룹 추가, 재로그인 |
| `robot moved during gyro bias calibration` | 보정 중 움직임 감지. 로봇을 가만히 두면 자동으로 다시 수집 |
| `BMI160 read failed` 반복 | 선 접촉 불량 가능. 100회 연속 실패마다 자동 재초기화를 시도한다 |
| `calibrated_imu` 토픽이 없음 | 기본값 `use_kalman_filter: false`. [실행](#실행) 참고 |
| PC에서 토픽이 안 보임 | PC · Pi의 `ROS_DOMAIN_ID`(42) · `ROS_AUTOMATIC_DISCOVERY_RANGE`(SUBNET)가 같은지, 네트워크 연결 확인 |

---

## 패키지 구조

```text
lekiwi_imu/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_imu.yaml          # 파라미터
├── launch/lekiwi_imu.launch.py     # 실행 파일
├── include/lekiwi_imu/
│   ├── bmi160_driver.hpp           # BMI160 I2C 드라이버
│   ├── imu_math.hpp                # 단위 변환 · axis_map
│   ├── lekiwi_imu_node.hpp         # ROS 2 노드
│   ├── kalman_filter.hpp
│   ├── low_pass_filter.hpp
│   └── one_euro_filter.hpp
├── src/
│   ├── bmi160_driver.cpp
│   ├── lekiwi_imu_node.cpp          # main 포함
│   ├── kalman_filter.cpp
│   ├── low_pass_filter.cpp
│   └── one_euro_filter.cpp
└── test/
    └── test_filters.cpp            # 필터 시험 (ROS 없이)
```

## 참고한 코드

- **One Euro 필터**: Casiez · Roussel · Vogel, "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems", CHI 2012
- **LeKiwi `lekiwi_sensors/src/bmi160_node.cpp`**: I2C 직접 접근, 100 Hz · ±2 g · ±250 dps 설정, 자이로 바이어스 보정, `axis_map`, 공분산 값, 연속 읽기 실패 처리

## 검증 기록

- 2026-10-17, lekiwi04 (Pi 4, Jazzy): 빌드 경고 0, `offset_imu` · `lpf_imu` 각 99.98 Hz, 자이로 바이어스 `[-0.0076, -0.0020, -0.0076] rad/s`, 정지 시 roll −2.0° · pitch 4.9°
- 2026-10-18: 필터 세 개(지수 이동 평균 · One Euro · 칼만)를 새로 썼다. 정리 전 구현과 같은 입력에서 출력이 1e-12 안으로 같음을 `test_filters`로 확인한다
- RViz 자세 TF, 칼만 필터 출력, 실제 주행 중 자세 정확도는 아직 검증하지 않았다.
