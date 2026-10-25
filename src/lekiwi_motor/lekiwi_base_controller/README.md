# lekiwi_base_controller

LeKiwi 옴니 휠 베이스의 **`cmd_vel` → 바퀴 속도 변환, 바퀴 엔코더 → `odom`, 모터 안전 상태기계**를 맡는 노드 패키지.
바퀴 모터와는 `lekiwi_motor_driver_node`의 토픽 · 서비스로만 이야기하고, **시리얼 포트를 직접 열지 않는다.**

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_motor/lekiwi_base_controller`
- 메시지: `lekiwi_msgs/MotorStateArray`(구독), `lekiwi_msgs/WheelCommand`(발행)
- 이름 기준: `src/docs/interfaces.md`

> ⚠️ **실물 주행 검증 전이다.** PC에서 단위 테스트와 가짜 드라이버 통합 테스트만 통과했다.
> 처음 실물로 돌릴 때는 **바퀴를 바닥에서 띄우고** 저속(`linear.x` 0.03 m/s 이하)으로 바퀴 ID · 회전 방향부터 확인한다.
>
> 기구학 · 오도메트리 · 안전 처리 로직은 LeKiwi 저장소 `lekiwi_node`(브랜치 `feature/lekiwi-ros2`)에서 그대로 옮겼다.
> 바뀐 것은 모터 백엔드뿐이다 (Feetech 시리얼 직접 제어 → `lekiwi_motor_driver_node` 경유).

---

## 노드

- `lekiwi_base_controller_node`

드라이버와 함께 실행한다. 드라이버가 없으면 `lekiwi/motor/ready`는 계속 `false`이고 모터 전원 요청은 실패한다.

```text
lekiwi_motor_driver_node ── lekiwi/motor/states ──────→ lekiwi_base_controller_node ── odom, TF odom→base_footprint
                         ←─ lekiwi/motor/wheel_command ─        ↑ cmd_vel
                         ←─ lekiwi/motor/wheel_torque (호출) ─   ↕ lekiwi/motor/power, lekiwi/motor/ready
```

## 구독 (Subscribe)

| 토픽 | 타입 | QoS | 내용 |
|------|------|-----|------|
| `cmd_vel` | `geometry_msgs/Twist` | 기본 | 몸체 속도 명령. 0.3 s 넘게 끊기면 바퀴 명령 0 |
| `lekiwi/motor/states` | `lekiwi_msgs/MotorStateArray` | SensorData | 바퀴 엔코더 · 연결 상태 (`motor_backend: driver`일 때만) |

## 발행 (Publish)

| 토픽 | 타입 | 주기 · QoS | 내용 |
|------|------|-----------|------|
| `odom` | `nav_msgs/Odometry` | 50 Hz | 바퀴 회전량 기반 상대 이동 추정 (지도 기준 위치가 아니다) |
| TF `odom → base_footprint` | – | 50 Hz | 이 변환의 발행자는 이 노드 하나뿐이다 |
| `lekiwi/motor/wheel_joint_states` | `sensor_msgs/JointState` | 50 Hz | `left_wheel_joint`, `back_wheel_joint`, `right_wheel_joint` 속도 · 각도 |
| `lekiwi/motor/ready` | `std_msgs/Bool` | 바뀔 때, reliable + transient_local, 깊이 1 | `true`일 때만 바퀴가 움직인다 |
| `lekiwi/motor/wheel_command` | `lekiwi_msgs/WheelCommand` | 50 Hz, 모터 전원이 켜졌을 때만 | `ids = wheel_ids`, `velocity = 바퀴 속도 × wheel_directions` [rad/s] |

- 관절 속도(`velocity`)는 `odom_source: encoder`이고 새 엔코더 샘플이 있으면 엔코더 값, 아니면 명령 값이다.
- 관절 각도(`position`)는 position 피드백의 바퀴 위치 증분을 적분한 값이다 (시작 0, [-π, π]로 감음, `reset_odometry`와 무관).
  speed 피드백이나 mock 백엔드에서는 0에 머문다. 관절 축 부호는 RViz · 실물로 아직 확인하지 않았다.

## 서비스 (Service)

| 이름 | 타입 | 동작 |
|------|------|------|
| `lekiwi/motor/power` | `std_srvs/SetBool` | `true`: 드라이버에 토크 켜기 요청 → 새 엔코더 샘플이 오면 `ready=true`. `false`: 속도 0 → 토크 끔 → 고장 해제 |
| `lekiwi/motor/reset_odometry` | `std_srvs/Trigger` | `odom`을 원점으로 |

**호출하는 서비스**: `lekiwi/motor/wheel_torque` (`std_srvs/SetBool`, 드라이버). 사람이 직접 부르지 않는다.

---

## 안전 처리

| 상황 | 결과 |
|------|------|
| `cmd_vel`이 0.3 s(`cmd_vel_timeout`) 넘게 끊김 | 바퀴 명령 0 |
| 몸체 · 바퀴 속도 제한 초과, NaN 명령 | 방향을 유지하며 같은 비율로 줄임. NaN은 0 |
| `ready=false` (전원 꺼짐 · 샘플 대기 · 고장) | 속도 0 명령만 보낸다 |
| 엔코더 샘플이 0.5 s(`encoder_sample_timeout`) 넘게 없음 | 속도 0 → 토크 끔 → 고장 고정 |
| states가 0.5 s(`driver_states_timeout`) 넘게 끊기거나, 한 바퀴가 0.5 s 넘게 `online=false` | 속도 0 → 토크 끔 → 고장 고정 |
| 한 메시지에서만 바퀴가 `online=false` | 그 읽기만 실패로 친다. 0.5 s 안에 돌아오면 계속 동작 |
| 모터 전원이 켜진 동안 states가 바퀴 `torque_enabled=false`를 보고 (드라이버 watchdog · 모터 보호로 드라이버가 스스로 끔) | 다음 states 한 개로 바로 연결 끊김으로 본다 → 속도 0 → 토크 끔 요청 → 고장 고정 |
| 토크 끄기를 드라이버가 확인해 주지 않음 | 바퀴 명령 발행을 멈춘다 → 드라이버 watchdog이 0.5 s에 속도 0, 3 s에 토크 끔 |
| 노드 종료 (Ctrl+C) | 속도 0을 한 번 보내고 명령을 멈춘다. 토크는 드라이버 watchdog이 끈다 |

- **고장은 `lekiwi/motor/power false`가 성공한 뒤에만 풀린다.** 고장 상태에서 `true`를 부르면
  `disable motor power before recovering a latched fault`로 거절한다.
- 토크는 `lekiwi/motor/power` 서비스로만 켠다. 파라미터 `torque_enable`은 `false`만 허용하고, `true`면 노드가 시작하지 않는다.
- 노드는 토크가 꺼진 상태로 시작한다 (`ready=false`).
- `torque_enabled`는 토크 켜기가 성공한 시각 + 0.1 s보다 늦게 찍힌(`header.stamp`) states부터 본다.
  켜기 직전에 발행된 메시지가 서비스 응답보다 늦게 도착할 수 있기 때문이다. 전원이 꺼져 있을 때는 보지 않는다.
- 실제 드라이버는 바퀴가 연속 3번 응답하지 않으면 스스로 토크를 끈다. 그러면 위 규칙으로 컨트롤러도
  0.5 s를 기다리지 않고 바로 고장 고정한다. 바퀴별 0.5 s 허용은 그보다 짧은 읽기 실패(1~2번)에만 해당한다.

## 파라미터

설정 파일: `config/lekiwi_base_controller.yaml` (실물용: `motor_backend: driver`, `enable_motor_write: true`).
설정 파일 없이 실행하면 기본값은 안전 쪽이다 (`motor_backend: mock`, `enable_motor_write: false` → 로그만 남김).

**기구학 · 속도 제한** (`lekiwi_node/param/base_controller.yaml`, `lekiwi_description/config/base_frame.yaml`과 같은 값)

| 이름 | 타입 | 실물 설정값 | 설명 |
|------|------|-------------|------|
| `wheel_radius` | double | `0.05` | [m] |
| `base_radius` | double | `0.13647` | [m] 중심 ~ 바퀴 |
| `kinematics_frame_x`, `_y`, `_yaw` | double | `0.007587843843`, `-0.010776931094`, `2.094379865161` | 원래 모터 기구학 원점을 `base_footprint`에서 본 자세 |
| `max_linear_x`, `max_linear_y`, `max_linear_speed` | double | `0.23`, `0.23`, `0.23` | [m/s] 2026-10-22 (사용자 요청 0.3 m/s를 바퀴 상한에 맞춘 값: 축 하나만 최대여도 바퀴 ≤ max_wheel_speed) |
| `max_angular_z` | double | `1.53` | [rad/s] 약 88°/s (사용자 요청 90°/s, 같은 검사의 최대 1.5376) |
| `max_wheel_speed` | double | `4.601942363656923` | [rad/s] = 드라이버 `speed_tick_limit` 3000 (STS3215 · LeRobot 상한) |

몸체 상한은 축 하나만 최대로 줘도 바퀴가 `max_wheel_speed` 안에 드는 값이어야 한다 (`ValidateBaseControllerParameters`가
시작할 때 검사한다). 그래서 상한 안의 한 축 명령은 줄지 않고 그대로 실행된다. 직선 + 회전처럼 섞은 명령이 바퀴 상한을 넘으면
`ScaleToWheelEnvelope`가 세 축을 같은 비율로 줄여 **방향을 지킨다**.
조작 도구 · Nav2는 자기 상한을 따로 쓴다 (`lekiwi_teleop` 키보드 · 터미널 도구는 기본 0.10 m/s, `--max-linear`로 0.20 m/s까지. Nav2는 0.08 m/s).
설정 파일 없이 실행할 때의 코드 기본값은 예전 값(0.11 · 0.1 · 0.11 m/s, 0.75 rad/s, tick 1500)이다.

**오도메트리 · 시간**

| 이름 | 타입 | 실물 설정값 | 설명 |
|------|------|-------------|------|
| `odom_frame`, `base_frame` | string | `odom`, `base_footprint` | |
| `cmd_vel_timeout` | double | `0.3` | [s] |
| `tf_time_offset` | double | `0.0` | [s] TF 시각만 앞당김 |
| `odom_source` | string | `encoder` | `encoder` \| `command` |
| `encoder_read_period` | double | `0.1` | [s] 엔코더 샘플 주기 |
| `encoder_sample_timeout` | double | `0.5` | [s] position 피드백이면 `3 × encoder_read_period` 이상 |

**모터 백엔드 · 엔코더**

| 이름 | 타입 | 실물 설정값 | 설명 |
|------|------|-------------|------|
| `motor_backend` | string | `driver` | `driver` \| `mock`. 그 밖의 값(예전 `feetech` 포함)은 mock |
| `enable_motor_write` | bool | `true` | `driver`와 함께 `true`여야 실제 바퀴 백엔드를 쓴다 |
| `wheel_ids` | int[] | `[8, 9, 7]` | 모델 관절 `left`, `back`, `right` 순서의 모터 ID (실물 위치: 뒤 · 오른쪽 · 왼쪽) |
| `wheel_directions` | double[] | `[1.0, 1.0, 1.0]` | 명령은 × 방향, 엔코더는 ÷ 방향 |
| `torque_enable` | bool | `false` | 반드시 `false` |
| `encoder_feedback_source` | string | `position` | `position`: tick 차분 (3구간 창 속도) \| `speed`: 모터 보고 속도 |
| `encoder_position_ticks_per_revolution` | double | `4096.0` | 위치 감김 주기 |
| `encoder_odom_scale` | double | `1.0` | 실측 odom 보정 계수 |
| `encoder_position_tick_deadband` | int | `0` | position 피드백에서는 0이어야 한다 |
| `encoder_speed_deadband` | double | `0.07669903939428206` | [rad/s] speed 피드백 불감대 (tick 50과 같음) |
| `log_encoder_reads` | bool | `false` | 엔코더 계산 값을 매 샘플 INFO 로그로 |
| `driver_states_timeout` | double | `0.5` | [s] states · 바퀴 online이 이보다 오래되면 연결 끊김 |
| `torque_service_timeout` | double | `1.0` | [s] 드라이버 토크 서비스 응답 대기 |

**토픽 · 서비스 이름**: `cmd_vel_topic`, `odom_topic`, `wheel_joint_states_topic`, `motor_ready_topic`, `motor_power_service`,
`reset_odometry_service`, `states_topic`, `wheel_command_topic`, `wheel_torque_service` (기본값은 위 표의 이름).

---

## 빌드

```bash
cd ~/lekiwi_ws
cbs lekiwi_msgs lekiwi_base_controller && sb
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나 `cb` / `cbs` / `cbsource`를 사용한다.

## 테스트

```bash
cd ~/lekiwi_ws
colcon test --packages-select lekiwi_base_controller --event-handlers console_direct+
colcon test-result --verbose
```

- 통합 테스트(`test_driver_integration`)는 가짜 드라이버를 프로세스 안에 띄우고, 테스트마다 다른 namespace와
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 쓴다. 실제 로봇 토픽에는 닿지 않는다. 약 12초 걸린다.

---

## 실행

```bash
# 터미널 1: 모터 드라이버 (바퀴 버스를 여는 유일한 노드)
ros2 launch lekiwi_motor_driver lekiwi_motor_driver.launch.py

# 터미널 2: 베이스 컨트롤러
ros2 launch lekiwi_base_controller lekiwi_base_controller.launch.py
```

정상 시작 로그:

```text
using motor driver: states /lekiwi/motor/states, commands /lekiwi/motor/wheel_command, torque /lekiwi/motor/wheel_torque (wheel ids [8, 9, 7])
motor backend: driver (real wheels); odom source: encoder (position feedback); publishing /odom, /lekiwi/motor/ready
```

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_base_controller/config/lekiwi_base_controller.yaml` | 파라미터 파일 |

**모터 없이 돌려 보기 (mock)**: `ros2 run lekiwi_base_controller lekiwi_base_controller_node` — 설정 파일 없이 실행하면
mock 백엔드라서 바퀴 명령을 보내지 않는다. `odom_source: encoder`이므로 `odom`은 움직이지 않는다.

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic echo --once /lekiwi/motor/ready` | 시작 직후 `false` |
| `ros2 service call /lekiwi/motor/power std_srvs/srv/SetBool "{data: true}"` | `success: true` 뒤 `ready`가 `true` |
| `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}}"` | **바퀴를 띄운 상태에서만.** `Ctrl+C` 뒤 0.3 s 안에 멈춤 |
| `ros2 topic echo /lekiwi/motor/wheel_command` | `ids: [8, 9, 7]`, 전원이 켜졌을 때만 나옴 |
| `ros2 topic hz /odom` | 약 50 Hz |
| `ros2 service call /lekiwi/motor/power std_srvs/srv/SetBool "{data: false}"` | 속도 0 → 토크 끔, `ready=false` |
| `ros2 service call /lekiwi/motor/reset_odometry std_srvs/srv/Trigger` | `odom`이 원점으로 |

---

## 처리 흐름

```text
50 Hz 제어 타이머 (control_group: 타이머 · cmd_vel · power · reset_odometry를 차례로 실행)
  ├─ (1) 안전 판단: 엔코더 마감 시간 확인 → 0.1 s마다 엔코더 샘플 → 연결 · 전원 확인
  │        끊김 · 시간 초과 → 속도 0 → 토크 끔 → 고장 고정 → ready=false
  ├─ (2) cmd_vel (0.3 s 지나면 0) → 몸체 · 바퀴 속도 제한 → 옴니 기구학 → 바퀴 속도
  ├─ (3) ready가 아니면 0 → WheelCommand (ids = wheel_ids, velocity × 방향)
  └─ (4) 엔코더 위치 증분 → odom · TF · 바퀴 관절

lekiwi/motor/states (states_group) → 최신 메시지 · 바퀴별 마지막 online 시각 · 전원이 켜진 동안 torque_enabled=false 기록
lekiwi/motor/wheel_torque 클라이언트 (torque_client_group) → 동기 호출 응답 처리
MultiThreadedExecutor 4 스레드: 제어 그룹이 토크 응답을 기다리는 동안 다른 스레드가 응답을 받는다
```

## 알려진 한계

- **실물 미검증**: 새 좌표계에서 바퀴 ID · 회전 방향(`wheel_directions`) · 관절 각도 부호를 실물로 확인하지 않았다.
- 토크 서비스 호출은 제어 타이머를 최대 `torque_service_timeout`(1 s) 멈출 수 있다 (드라이버가 응답하지 않을 때).
  그동안 `odom`도 나오지 않는다. 명령은 그 전에 0으로 보낸다.
- 컨트롤러는 시작할 때 토크를 직접 끄지 않는다 (`Connect()`는 아무것도 하지 않음). 시작 시 속도 0 · 토크 끔은
  드라이버가 맡는다. 이전 컨트롤러가 비정상 종료해 토크가 켜져 있었다면 드라이버 watchdog이 3 s 안에 끈다.
- 연결 끊김 처리와 엔코더 시간 초과가 같은 제어 주기에 겹치면 토크 끄기 요청이 두 번 갈 수 있다 (결과는 같다).
- 모터 전압 감시(예전 `monitor_motor_health`)는 이 노드에서 빠졌다. 모터 상태 비트 · 전압은 드라이버가 본다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `Package 'lekiwi_base_controller' not found` | 워크스페이스를 source하지 않음. `sb` |
| 빌드 중 `lekiwi_msgs`를 찾지 못함 | 메시지 패키지를 함께 빌드: `cbs lekiwi_msgs lekiwi_base_controller` |
| `torque_enable must be false` 로 시작 실패 | 설정 파일의 `torque_enable`을 `false`로. 토크는 `lekiwi/motor/power`로 켠다 |
| `motor backend: mock (logging only)` | 설정 파일이 적용되지 않음 또는 `motor_backend`/`enable_motor_write` 확인 |
| power `true`가 `failed to connect and enable motor power` | states가 안 오거나 바퀴가 offline. 드라이버 실행 · `ros2 topic hz /lekiwi/motor/states` 확인 |
| `wheel torque enable refused by motor driver: ...` | 드라이버가 거절한 이유가 뒤에 나온다 (바퀴 무응답, 속도 모드 아님, 보호 비트) |
| `service ... is not available` | 드라이버가 꺼져 있거나 namespace가 다름 |
| `motor driver link lost (...)` 후 `ready=false` | states 끊김, 바퀴 offline 0.5 s 이상, 또는 `driver reports torque off` (드라이버가 스스로 토크를 끔 — 드라이버 로그에서 이유 확인). 원인을 고친 뒤 power `false` → `true` |
| power `true`가 `disable motor power before recovering a latched fault` | 고장 고정 상태. power `false`를 먼저 부른다 |
| `encoder feedback timed out; ...` | 엔코더 샘플이 0.5 s 넘게 없음. 드라이버 로그 · 버스 상태 확인 |
| `cmd_vel exceeded body or wheel velocity limits and was limited` | 속도 제한을 넘는 명령. 제한값 안으로 보내거나 무시 |

---

## 패키지 구조

```text
lekiwi_base_controller/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_base_controller.yaml          # 실물용 파라미터
├── launch/lekiwi_base_controller.launch.py
├── include/lekiwi_base_controller/
│   ├── omni_kinematics.hpp                     # 3륜 옴니 기구학
│   ├── odometry.hpp                            # 평면 자세 적분
│   ├── motor_readiness.hpp                     # 준비 상태 래치
│   ├── motor_safety_coordinator.hpp            # 엔코더 · 전원 · 고장 안전 상태기계
│   ├── base_controller.hpp                     # 파라미터, 명령 제한 · 오도메트리 선택, 모터 인터페이스 기반 클래스
│   ├── wheel_feedback_estimator.hpp            # ROS 없는 엔코더 계산 (position · speed)
│   ├── driver_motor_interface.hpp              # lekiwi_motor_driver_node 백엔드
│   └── lekiwi_base_controller_node.hpp         # ROS 2 노드
├── src/                                        # 위 헤더의 구현 + lekiwi_base_controller_main.cpp
└── test/
    ├── test_omni_kinematics.cpp
    ├── test_motor_readiness.cpp
    ├── test_motor_safety_coordinator.cpp
    ├── test_base_motor_interface_contract.cpp
    ├── test_base_node_smoke.cpp                # 계산 함수 · 파라미터 검증 · 소스 계약
    ├── test_wheel_feedback_estimator.cpp       # 엔코더 계산 · 메시지 변환
    └── test_driver_integration.cpp             # 가짜 드라이버와 노드 전체
```

## 참고한 코드

- **LeKiwi 저장소 `lekiwi_node`** (`feature/lekiwi-ros2`): 기구학, 오도메트리, 준비 래치, 안전 조정기, 파라미터 검증,
  테스트. 함수 이름만 규칙서에 맞게 PascalCase로 바꿨다
- **LeKiwi 저장소 `lekiwi_node/src/feetech_motor_interface.cpp`**: position 피드백 계산 (첫 샘플 기준점, 반 바퀴 감기,
  3구간 창 속도), speed 불감대
- **lekiwi_motor_driver**: 토픽 · 서비스 이름, QoS, 패키지 · README 형식

## 검증 기록

- 2026-10-18, PC (Ubuntu 24.04, Jazzy): 빌드 경고 0 (`-Wall -Wextra -Wpedantic`). `colcon test` 7개 테스트 실행 파일 모두 통과
  (가짜 드라이버 통합 테스트 포함: power → ready → 주행 명령 · 부호 → cmd_vel 끊김 0 → states 끊김 고장 · 토크 끔 →
  power false 전 복구 거절, 바퀴 일시 offline 무고장 · 장시간 offline 고장, 드라이버 쪽 토크 꺼짐 고장 ·
  켜기 전 메시지 무시, 서비스 교착 없음)
- 2026-10-18, lekiwi04 (Pi 4, Jazzy): 빌드 경고 0, `colcon test` 통과. `lekiwi_bringup` 통합 시험(실제 드라이버 + 가상 Feetech 버스)에서
  power → ready → 전진 · 옆 · 회전 odom 왕복, `cmd_vel` 끊김 정지, 컨트롤러 강제 종료 시 드라이버 단독 정지, 드라이버 종료 시 고장 고정 통과
- 2026-10-18, lekiwi04 실물 (사람 없이, **토크를 켜지 않음**): 실제 드라이버 · 모터와 기동, `/odom` 50 Hz, `lekiwi/motor/wheel_joint_states` 50 Hz,
  `lekiwi/motor/ready` false, 정지 상태 odom (0, 0, 0), CPU 약 12 % (코어 하나 기준)
- 실물 바퀴 토크 · 주행, 바퀴 ID · 방향 · 관절 각도 부호는 아직 확인하지 않았다 ([현장 확인 절차](../../docs/field_validation.md) 2단계).
