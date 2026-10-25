# lekiwi_bringup

LeKiwi **Pi 브링업**: 로봇 모델 · 센서 · 모터 드라이버 · 베이스 컨트롤러를 한 번에 켜는 launch와 장비별 설정, PC용 RViz launch.
이름 규칙은 [이름 계약](../docs/interfaces.md), 실물 확인 순서는 [현장 확인 절차](../docs/field_validation.md)를 따른다.

> ✅ 바퀴 토크는 **꺼진 채** 시작한다. 드라이버가 시작할 때 남아 있던 바퀴 토크 · 속도도 정리한다.
> 바퀴를 움직이려면 `lekiwi/motor/power`(조작 도구가 부른다)로 켠다.
>
> ✅ 팔 토크는 **바꾸지 않는다** (켜져 있던 팔이 떨어지지 않게). 팔은 `lekiwi/motor/arm_torque`(게이트웨이가 부른다)로 켠 뒤에만 움직인다.
> ⚠️ 2026-10-22 지금 웹 비상 정지는 아직 팔 토크를 끄지 않는다 (2단계에서 붙인다). 그때까지 실물에서 `arm_torque` true를 시험 밖에서 부르지 않는다.
>
> ⚠️ 같은 모터 버스를 쓰는 저장소 `lekiwi_node` 브링업이나 LeRobot 호스트와 동시에 실행하지 않는다.

---

## launch

### `bringup.launch.py` (Pi)

| 섹션 | 노드 | 켜는 인자 |
|------|------|----------|
| 1. 로봇 모델 | `robot_state_publisher` (xacro, `use_hardware_sensors:=true`), `joint_state_publisher` | 항상 |
| 2. 센서 | `lekiwi_imu_node` | `use_imu` |
| | `lekiwi_lidar_node`, `lekiwi_scan_filter_node` | `use_lidar` |
| | `front_camera_node`, `wrist_camera_node` | `use_camera` (`lekiwi_camera`가 빌드돼 있을 때만) |
| 3. 모터 | `lekiwi_motor_driver_node` | `use_motors` |
| | `lekiwi_base_controller_node` (+ 드라이버 바퀴 쓰기) | `use_motors` 그리고 `use_base_controller` |
| | 드라이버 팔 쓰기 + `lekiwi_lerobot` 팔로워 팔 `lekiwi_follower_arm_node` | `use_motors` 그리고 `use_arm`. 팔로워는 `lekiwi_lerobot`이 빌드돼 있고 `lerobot_python`이 있을 때만 (없으면 로그만 남기고 건너뛴다) |

팔로워 팔 노드는 `lekiwi_lerobot/launch/follower_arm.launch.py`와 같은 노드를 같은 방법(`lerobot_python`을 prefix로)으로 켜되,
다른 노드처럼 파라미터를 [`lekiwi_lerobot/config/follower_arm.yaml` → 장비 프로파일 → (있으면) `~/lekiwi_ws/arm_profile.yaml` → (주면) `robot_id` 인자] 순서로 넣는다.
그래서 로봇마다 다른 보정 이름은 장비 프로파일의 `lekiwi_follower_arm_node: ros__parameters: robot_id: '04'` 식으로 둘 수 있다.
`arm_profile.yaml`은 웹 게이트웨이(보정 탭)가 `lekiwi_follower_arm_node`의 `robot_id` · `rest_pose_ticks`를 통째로 쓰는 파일이다. 사람이 쓴 장비 프로파일은 고치지 않고 그 위에 얹는다.
없으면 `arm profile: … not found; skipping it` 로그만 남기고 건너뛴다.
팔로워 팔 노드가 죽어도 bringup은 계속 돈다 (on_exit 종료를 걸지 않는다). 드라이버는 마지막 팔 목표를 유지한다.

`joint_state_publisher`는 바퀴 관절을 `lekiwi/motor/wheel_joint_states`에서 받고, 팔 관절은 0(손목 회전은 -π/2)으로 채운다. 팔 각도 보정 전이다.
발행 주기는 `joint_state_rate`(기본 10 Hz)다. 주행에 쓰는 `odom → base_footprint`는 베이스 컨트롤러가 따로 내므로 이 값과 상관없다.

**인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_profile` | (비움) | 장비별 설정 파일. 비우면 `~/lekiwi_ws/robot_profile.yaml`, 없으면 `config/robot_profile.yaml` |
| `use_imu` | `true` | BMI160 IMU. IMU가 없는 장비는 `false` (PC Cartographer도 IMU 없는 설정으로) |
| `use_lidar` | `true` | YDLIDAR와 자기 반사 필터 |
| `use_camera` | `false` | 전방 · 손목 카메라. CPU를 더 쓴다 |
| `use_motors` | `true` | 모터 버스 드라이버 (모터 9개 읽기) |
| `use_base_controller` | `true` | 바퀴 쓰기 + 베이스 컨트롤러. `false`면 모터는 읽기 전용 |
| `use_arm` | `true` | 드라이버 팔 쓰기(`arm_write_enabled`) + 팔로워 팔. `false`면 팔은 읽기만 한다 |
| `lerobot_python` | `~/lerobot060_venv/bin/python` | 팔로워 팔을 실행할 lerobot 0.6.0 가상환경 python (노드 prefix). 없으면 팔로워를 건너뛴다 |
| `robot_id` | `''` | 팔로워 보정 파일 이름 `<robot_id>.json`. 비우면 넘기지 않는다 (장비 프로파일 → 팔로워 설정, 비면 호스트 이름) |
| `joint_state_rate` | `10` | [Hz] 관절 상태 · 관절 TF 발행 주기 (화면 표시용). 50 Hz일 때 `joint_state_publisher`가 CPU 33%를 쓰고, `/tf`를 듣는 Nav2 노드 모두가 부담을 나눠 졌다 |

```bash
ros2 launch lekiwi_bringup bringup.launch.py                              # 전체
ros2 launch lekiwi_bringup bringup.launch.py use_base_controller:=false   # 모터 읽기 전용 (1단계)
ros2 launch lekiwi_bringup bringup.launch.py use_camera:=true             # 카메라 포함
ros2 launch lekiwi_bringup bringup.launch.py use_arm:=false               # 팔은 읽기만 (팔로워 없음)
```

### `rviz.launch.py` (PC)

`lekiwi_description/rviz/base.rviz`로 RViz를 연다: 고정 프레임 `odom`, 로봇 모델 · TF · `/odom` · `/lekiwi/sensor/scan_filtered`.

```bash
ros2 launch lekiwi_bringup rviz.launch.py
```

---

## 파라미터 순서와 장비 프로파일

노드마다 파라미터를 다음 순서로 넣고, **뒤에 넣은 값이 이긴다.**

1. 각 패키지의 기본 설정 (`<패키지>/config/<패키지>.yaml`)
2. (베이스 컨트롤러만) `lekiwi_description/config/base_frame.yaml` — 기구학 원점
3. 장비 프로파일
4. (팔로워 팔만, 있으면) `~/lekiwi_ws/arm_profile.yaml` — 게이트웨이가 쓴다. 장비 프로파일의 같은 키를 덮는다
5. (팔로워 팔만, 주면) `robot_id` launch 인자

장비 프로파일은 로봇마다 다른 값(포트, 바퀴 ID · 방향 · 치수, odom 보정, IMU 주소)만 둔다. 워크스페이스 `src` 밖에 있어 git에 섞이지 않는다.

```bash
cp ~/lekiwi_ws/src/lekiwi_bringup/config/robot_profile.yaml ~/lekiwi_ws/robot_profile.yaml
nano ~/lekiwi_ws/robot_profile.yaml
```

| 노드 | 키 | 기본값 | 바꿀 때 |
|------|----|--------|---------|
| `lekiwi_motor_driver_node` | `port` | `/dev/ttyACM0` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_...-if00` 권장 |
| | `wheel_ids` | `[7, 8, 9]` | 바퀴 모터 ID가 다른 조립품 |
| `lekiwi_base_controller_node` | `wheel_ids` | `[8, 9, 7]` | 모델 바퀴 left · back · right 순서의 ID (2단계 시험으로 확인) |
| | `wheel_directions` | `[1.0, 1.0, 1.0]` | 바퀴가 반대로 돌 때 |
| | `wheel_radius`, `base_radius` | `0.05`, `0.13647` | 치수가 다른 조립품 |
| | `encoder_odom_scale` | `1.0` | 3단계 거리 시험 결과 |
| `lekiwi_lidar_node` | `port` | `/dev/ttyUSB0` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_...` 권장 |
| `lekiwi_imu_node` | `i2c_address`, `axis_map` | `104`, `[1, 2, 3]` | SA0를 3.3V에 연결한 모듈은 `105` |
| `lekiwi_follower_arm_node` | `rest_pose_ticks` | `[]` (없음) | 팔 접은 자세 [tick, ID 1~6]. 없으면 `lekiwi/arm/rest`를 거절한다. 팔을 접고 `lekiwi/motor/states`의 `position_ticks`를 적는다 (예: 이 파일의 lekiwi04 주석, 4단계 실물 시험에서 다시 잰다) |

드라이버의 `wheel_ids`와 베이스 컨트롤러의 `wheel_ids`는 같은 모터들이어야 한다 (순서는 달라도 된다).

---

## 빌드 · 시험

```bash
cd ~/lekiwi_ws
cbs lekiwi_bringup && sb
colcon test --packages-select lekiwi_bringup --event-handlers console_direct+ && colcon test-result --verbose
```

처음 쓰는 Pi에는 `sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher`가 필요하다.

- `test_bringup_launch`: 인자별 노드 구성, 드라이버 바퀴 쓰기가 베이스 컨트롤러와 함께만 켜지는지, 드라이버 팔 쓰기가 `use_arm`을 따르는지, 팔로워가 패키지 · `lerobot_python`이 있을 때만 들어가는지, 팔로워 파라미터 순서 · `robot_id`는 줄 때만, `arm_profile.yaml`이 있을 때만 장비 프로파일 뒤 · `robot_id` 앞에 들어가는지 (없으면 로그) · 실제 rclpy에서 뒤의 값이 이기는지, 실제 팔로워 노드가 `lerobot_python` prefix로 돌고 on_exit 종료가 없는지 · `lekiwi_lerobot` 단독 launch와 같은 노드인지, 베이스 바퀴 상한 × `speed_tick_scale` = 드라이버 `speed_tick_limit`(3000)인지, 파라미터 순서, 카메라 패키지가 없을 때 건너뛰는지, 프로파일 선택 순서
- `test_motor_stack_pty`: 모터 드라이버(설정 파일 그대로) + 베이스 컨트롤러를 **가상 Feetech 버스**에 붙여 `power → ready → cmd_vel → 바퀴 → 엔코더 → odom` 왕복(전진 · 옆 · 회전, 0.2 m/s 직진이 tick 1500을 넘어 그대로 나오는지), `cmd_vel` timeout 정지, 전원 끔, 베이스 컨트롤러 강제 종료 시 드라이버 단독 정지, 드라이버 종료 시 고장 고정. 하드웨어 없음
- `test_motor_stack_pty`가 쓰는 ROS 도메인은 기본으로 프로세스 번호에서 고른다 (60~79). `LEKIWI_TEST_ROS_DOMAIN_ID`로 정할 수 있다

## 확인 명령어 (Pi에서 bringup 실행 중)

| 명령어 | 정상 |
|--------|------|
| `ros2 node list` | 위 표의 노드들 |
| `ros2 topic hz /lekiwi/sensor/scan_filtered` | 약 10 Hz |
| `ros2 topic hz /lekiwi/sensor/offset_imu` | 약 100 Hz |
| `ros2 topic hz /lekiwi/motor/states` | 약 30 Hz |
| `ros2 topic hz /joint_states` | 약 10 Hz (`joint_state_rate`) |
| `ros2 topic echo --once /lekiwi/motor/ready` | 토크를 켜기 전 `data: false` |
| `ros2 run tf2_ros tf2_echo base_footprint lidar_link` | 약 (-0.001, 0.002, 0.025) m |

노드는 보이는데 토픽이 비면 `ros2 daemon stop` 후 다시 본다 (다른 설정으로 떠 있던 데몬).

## 패키지 구조

```text
lekiwi_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/robot_profile.yaml    # 장비별 설정 예시 (~/lekiwi_ws/robot_profile.yaml로 복사)
├── launch/
│   ├── bringup.launch.py        # Pi
│   └── rviz.launch.py           # PC
└── test/
    ├── test_bringup_launch.py
    └── test_motor_stack_pty.py
```

## 검증 기록

- 2026-10-18, lekiwi04 (Pi 4, Jazzy), 사람 없이 · 움직임 없음, `use_base_controller:=false`:
  노드 6개 기동, 모터 9/9 응답(`/lekiwi/motor/states` 20 Hz), 라이다 Tmini Plus 10 Hz(첫 스캔 430개 중 346개 유효, 가장 가까운 0.11 m),
  스캔 필터 10 Hz(첫 스캔 258개 남김), IMU 100 Hz, `joint_states` 50 Hz, TF `base_footprint → lidar_link` = (-0.001, 0.002, 0.025) m
- 2026-10-18, lekiwi04, 사람 없이 · **토크를 켜지 않음**, 기본 인자(베이스 컨트롤러 포함): 노드 7개 기동,
  드라이버 `startup: wheel speed zeroed and torque disabled`, `/odom` · 바퀴 관절 50 Hz, `lekiwi/motor/ready` false,
  서비스 `lekiwi/motor/power` · `reset_odometry` · `wheel_torque`, TF `odom → base_footprint` (0, 0, 0),
  CPU(노드별, 코어 하나 기준) 베이스 컨트롤러 12 %, IMU 7 %, 드라이버 6 %, robot_state_publisher 5 %, 라이다 5 %, 스캔 필터 3 %
- 2026-10-18, lekiwi04: `colcon test` 전체 380개 통과 (`python3-tk` 설치 후)
- 아직 하지 않은 것: RViz에서 실제 물체와 스캔 방향 비교, 바퀴 토크 · 주행
