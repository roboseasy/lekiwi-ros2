# 노드 · 토픽 · 서비스 · 프레임 계약

이 워크스페이스 패키지들이 서로 주고받는 이름의 기준이다. 이름을 바꿀 때는 이 표를 먼저 고치고, 쓰는 쪽을 모두 함께 고친다.

## 이름 규칙

- LeKiwi 전용 토픽 · 서비스는 상대 이름 `lekiwi/<계층>/<이름>`을 쓴다. 계층: `sensor`, `motor`, `arm` (팔로워 팔, LeRobot 단위), `lerobot` (lerobot 데이터셋과 같은 뜻의 값), `zmq_host` (PC ZMQ 세션), `safety` (게이트웨이 비상 정지)
- Cartographer · Nav2 · RViz와 맞닿는 표준 인터페이스는 표준 이름을 그대로 쓴다: `cmd_vel`, `odom`, `joint_states`, `tf`, `tf_static`, `map`
- 외부 스택에 LeKiwi 토픽을 넘길 때는 launch remap으로 연결한다 (예: Cartographer `scan` ← `lekiwi/sensor/scan_filtered`)
- 모두 상대 이름이므로 launch에서 namespace를 주면 로봇별로 나뉜다. 기본은 namespace 없음

## 노드 (실행 위치)

| 노드 | 패키지 | 위치 | 하는 일 |
|------|--------|------|---------|
| `lekiwi_motor_driver_node` | `lekiwi_motor_driver` | Pi | Feetech 버스(`/dev/ttyACM0`)를 여는 유일한 노드. 모터 9개 읽기, 바퀴 3개 · 팔 6개 쓰기 |
| `lekiwi_base_controller_node` | `lekiwi_base_controller` | Pi | `cmd_vel` → 바퀴 속도, 바퀴 → `odom`, 모터 안전 상태기계 |
| `lekiwi_imu_node` | `lekiwi_imu` | Pi | BMI160 |
| `lekiwi_lidar_node` | `lekiwi_lidar` | Pi | YDLIDAR |
| `lekiwi_scan_filter_node` | `lekiwi_scan_filter` | Pi | 로봇 자기 반사 제거 |
| `front_camera_node`, `wrist_camera_node` | `lekiwi_camera` | Pi | 전방 · 손목 카메라 |
| `robot_state_publisher`, `joint_state_publisher` | (apt) | Pi | URDF → TF |
| `lekiwi_zmq_host_node` | `lekiwi_zmq_host` | Pi (시스템 python) | lerobot LeKiwi 호스트와 같은 ZMQ(명령 `PULL :5555` · 관측 `PUSH :5556`, CONFLATE) ↔ ROS. PC는 ROS 없이 lerobot `LeKiwiClient`로 붙는다. 팔 → `lekiwi/arm/goal`, 바퀴 → lerobot 기구학 → 베이스 기구학의 역 → `cmd_vel`. 번역만 하고 팔로워 · 베이스 컨트롤러 안전층을 그대로 거친다 (`lekiwi_bringup` `use_zmq_host:=true`, 기본) |
| Cartographer · Nav2 · 조작 도구 | `lekiwi_cartographer`, `lekiwi_navigation2`, `lekiwi_teleop` | Pi | 지도 · 경로 · 조작 (키보드 창은 `ssh -X`로 PC 화면에 뜬다) |
| RViz | (apt) | PC | 보기 · 초기 위치 · 목표 지정. Nav2 감독 도구가 이 노드(`rviz` · `rviz2`)를 조작 화면으로 지켜본다 |

## 토픽

| 이름 | 타입 | 발행 | 구독 | QoS · 주기 |
|------|------|------|------|-----------|
| `lekiwi/motor/states` | `lekiwi_msgs/MotorStateArray` | 모터 드라이버 | 베이스 컨트롤러, 팔로워 팔, 진단 | SensorData, 30 Hz (lerobot LeKiwi 데이터셋 30 fps) |
| `lekiwi/motor/wheel_command` | `lekiwi_msgs/WheelCommand` | 베이스 컨트롤러 | 모터 드라이버 | reliable, 깊이 1, 50 Hz (모터 전원이 켜졌을 때만) |
| `lekiwi/motor/arm_command` | `lekiwi_msgs/ArmCommand` | 팔로워 팔 (`lekiwi_lerobot`) | 모터 드라이버 | reliable, 깊이 1, 약 30 Hz. 팔 ID별 목표 위치 [tick 0~4095]. 드라이버는 받을 때마다 메시지의 모든 ID를 SYNC_WRITE 한 번으로 쓴다 (ACK가 없어 걸러 내지 않는다). 모터마다 마지막 위치 ± `arm_max_goal_offset_ticks`(1024 = 90°) 안으로 자른다. 팔로워는 따라가는 중 리더 목표를 그대로 보낸다 (지난 명령에서 한 번에 `follow_step_deg` 45°까지) |
| `lekiwi/motor/arm_ready` | `std_msgs/Bool` | 모터 드라이버 | 팔로워 팔, 게이트웨이 | reliable + transient_local, 깊이 1. 바뀔 때마다와 시작할 때 한 번. true = 드라이버가 팔 명령을 받는 중 (`arm_torque` true 성공 뒤). 시작 · `arm_torque` false · 켜기 실패 때 false |
| `lekiwi/motor/arm_registers` | `lekiwi_msgs/ArmRegisters` | 모터 드라이버 | 팔로워 팔 (보정 파일과 비교) | reliable + transient_local, 깊이 1. 시작할 때 한 번, 응답하지 않던 팔을 나중에 읽으면 다시. 팔 EEPROM의 `Homing_Offset`(부호 비트 11을 푼 값) · 위치 한계 · P/D/I. 읽기만 한다 |
| `lekiwi/lerobot/state` | `lekiwi_msgs/LerobotState` | 팔로워 팔 (`lekiwi_lerobot`) | PC 데이터셋 기록기 | reliable, 깊이 10, 모터 상태마다 (30 Hz). lerobot LeKiwi `observation.state` 9개: `names` = `arm_*.pos` 6개 (LeRobot 각도 · 그리퍼 0~100) + `x.vel` · `y.vel` [m/s] · `theta.vel` [deg/s] (바퀴 `Present_Velocity`로 lerobot `_wheel_raw_to_body`, odom이 아니다). stamp는 모터 상태의 stamp. 팔 6개 · 바퀴 3개가 그 샘플에서 모두 응답하고, 모터 ID가 lerobot LeKiwi와 같고, 팔 보정이 `arm_registers`와 맞을 때만 |
| `cmd_vel` | `geometry_msgs/Twist` | 조작 창 · 게이트웨이 · ZMQ 호스트(세션 active일 때만 발행자가 있다) 또는 Nav2 (`collision_monitor`) 중 하나 | 베이스 컨트롤러 | 기본 |
| `lekiwi/arm/goal` | `sensor_msgs/JointState` | 게이트웨이, ZMQ 호스트 | 팔로워 팔 | SensorData. 이름 `arm_shoulder_pan` … `arm_gripper` (`.pos` 없음), LeRobot 단위 (몸 관절 [deg], 그리퍼 0~100). 6개가 모두 있어야 받는다 |
| `lekiwi/arm/status` | `std_msgs/String` (JSON) | 팔로워 팔 | 게이트웨이, ZMQ 호스트 | reliable, 깊이 10, 30 Hz. `{state, reason, robot_id, diffs, ramp_progress, warnings}` (`lekiwi_lerobot` README) |
| `lekiwi/zmq_host/status` | `std_msgs/String` (JSON) | ZMQ 호스트 | 게이트웨이 | reliable + transient_local, 깊이 1. 바뀔 때 + 1 Hz. `{session: idle\|active\|blocked, reason, clients, client_ip, estop_unknown}` |
| `lekiwi/safety/estop_latched` | `std_msgs/Bool` | 게이트웨이 | ZMQ 호스트 | reliable + transient_local, 깊이 1. 비상 정지가 걸려 있으면 true (걸리면 조작 화면 노드가 사라져 노드만으로는 알 수 없다). 발행자가 없으면 ZMQ 호스트는 '모름'으로 보고 거절하지 않는다 (`estop_unknown`) |
| `odom` | `nav_msgs/Odometry` | 베이스 컨트롤러 | Cartographer, Nav2 | 기본, 50 Hz |
| `lekiwi/motor/ready` | `std_msgs/Bool` | 베이스 컨트롤러 | 조작 창, Nav2 감독 | reliable + transient_local, 깊이 1 |
| `lekiwi/motor/wheel_joint_states` | `sensor_msgs/JointState` | 베이스 컨트롤러 | `joint_state_publisher` | 기본 |
| `joint_states` | `sensor_msgs/JointState` | `joint_state_publisher` | `robot_state_publisher` | 기본 |
| `lekiwi/sensor/scan` | `sensor_msgs/LaserScan` | 라이다 | 스캔 필터 | SensorData, 10 Hz, frame `lidar_link` |
| `lekiwi/sensor/scan_filtered` | `sensor_msgs/LaserScan` | 스캔 필터 | Cartographer, Nav2 | SensorData, frame `lidar_link`. 제외한 빔은 NaN |
| `lekiwi/sensor/offset_imu` | `sensor_msgs/Imu` | IMU | Cartographer | 자이로 바이어스만 제거, 자세 없음 |
| `lekiwi/sensor/lpf_imu`, `lekiwi/sensor/calibrated_imu` | `sensor_msgs/Imu` | IMU | — | 필터 + 자세 |
| `lekiwi/sensor/front_camera/…`, `lekiwi/sensor/wrist_camera/…` | 영상 | 카메라 | — | frame `front_camera_optical_frame`, `wrist_camera_optical_frame` |

## 서비스

| 이름 | 타입 | 서버 | 호출하는 쪽 | 동작 |
|------|------|------|-------------|------|
| `lekiwi/motor/power` | `std_srvs/SetBool` | 베이스 컨트롤러 | 조작 창, Nav2 감독, ZMQ 호스트 (세션 시작 true · 끝 false) | true: 바퀴 토크를 켜고 엔코더가 들어오면 `ready=true`. false: 속도 0, 토크 끔, 고장 해제 |
| `lekiwi/motor/reset_odometry` | `std_srvs/Trigger` | 베이스 컨트롤러 | 사용자 | odom을 원점으로 |
| `lekiwi/motor/wheel_torque` | `std_srvs/SetBool` | 모터 드라이버 | **베이스 컨트롤러만** | true: 바퀴가 속도 모드인지 확인 → 속도 0 → 토크 켬(ACK 확인). false: 속도 0 → 토크 끔. 팔 ID는 건드리지 않는다 |
| `lekiwi/arm/rest` | `std_srvs/Trigger` | 팔로워 팔 (`lekiwi_lerobot`) | 게이트웨이 (bringup 끄기 전) | **부르기 전에 리더 목표(`lekiwi/arm/goal`)를 멈춘다**: 접은 자세 중 이상이 생기면 (걸어 두지 않는 모터 상태 끊김 · 응답 없음도) 접은 자세 모드에서 나오고 다시 들어가지 않으며, 이상이 풀린 뒤 오는 리더 목표를 따라간다. 리더 목표를 무시하고 잰 자세 → 접은 자세(`rest_pose_ticks`, 로봇마다 장비 프로파일 또는 게이트웨이가 쓰는 `~/lekiwi_ws/arm_profile.yaml`에 둔다 (bringup이 장비 프로파일 뒤에 얹는다). 기본은 없음 = 거절. 보정 범위로 자름: lekiwi04 팔꿈치 3190은 편 팔에서 부르면 범위 끝 3094에서 멈추고, 이미 접혀 범위 밖에 있으면 그 자리에 둔다)로 `rest_time_s`(3 s) smoothstep 램프, 한 주기 이동 제한 그대로. `lekiwi/arm/status`가 `resting`(`ramp_progress`) → 모든 관절이 `rest_tolerance_deg`(3°) 안이면 `rested` (램프 뒤 1초 넘게 못 오면 `warnings`에 관절 이름을 적고 `rested`). `rested` 뒤에는 명령을 내지 않는다 (드라이버가 유지). 게이트웨이는 `rested`를 본 뒤 `lekiwi/motor/arm_torque` false를 부른다. 거절(`success: false`, 한국어 이유): 보정 · lerobot을 못 불러옴, `arm_ready` false, 모터 · 레지스터 이상 · 상태 끊김, 접은 자세 없음. `arm_ready` 메시지 · 이상이 오면 접은 자세 모드에서 나온다 |
| `lekiwi/motor/arm_torque` | `std_srvs/SetBool` | 모터 드라이버 | 게이트웨이 (시작 · 팔 힘 빼기 · 비상 정지), ZMQ 호스트 (세션의 첫 팔 명령 때 true만) | true: 이미 `arm_ready`이고 모든 팔 토크가 켜져 있고 보호 비트가 없으면 아무것도 쓰지 않고 성공 (`arm_ready`도 다시 내지 않는다). 아니면 팔마다 다시 읽어 응답 · 위치 모드(0) · 보호 비트 확인 → 꺼져 있던 팔만 `Goal_Position` = 지금 위치(ACK) → 토크 켬(ACK), 실패한 ID는 한 번 더. 그래도 실패하면 **호출 전에 꺼져 있던 팔만** 다시 끄고(잡고 있던 팔은 그대로) `success: false`와 ID. false: 팔 ID에만 하나씩 토크 끔(ACK, 브로드캐스트 없음), 확인 못 한 ID는 1초마다 다시. 바퀴는 건드리지 않는다 |

사람이 바퀴 토크를 켤 때는 드라이버 서비스가 아니라 `lekiwi/motor/power`를 부른다. 베이스 컨트롤러의 안전 상태기계를 거치게 하기 위해서다.

팔 토픽 · 서비스(`arm_command`, `arm_torque`, `arm_ready`, `arm_registers`)는 드라이버 파라미터 `arm_write_enabled: true`일 때만 생긴다 (`lekiwi_bringup`의 `use_arm:=true`, 기본). 없으면 팔 명령을 받지 않는다는 뜻이다. 이름은 드라이버 파라미터 `arm_command_topic` · `arm_torque_service` · `arm_ready_topic` · `arm_registers_topic`, 쓰기를 허용하는 팔은 `arm_ids`(기본 1~6, `wheel_ids`와 겹치지 않는다), 목표 제한은 `arm_max_goal_offset_ticks`(기본 1024 = 90°)다. `arm_*` 값은 팔 쓰기를 켰을 때만 검증한다. 팔 토크는 이 서비스로만 켜고 끈다. 팔 단위(LeRobot 각도) · 보정은 팔로워 팔만 다룬다.

## TF

| 변환 | 발행 | 비고 |
|------|------|------|
| `map → odom` | Cartographer(지도 작성) 또는 AMCL(저장 지도) 중 하나 | 둘을 동시에 띄우지 않는다 |
| `odom → base_footprint` | 베이스 컨트롤러 | 이 변환의 발행자는 하나뿐이다 |
| `base_footprint → …` (로봇 몸체 · 센서 · 카메라) | `robot_state_publisher` | `lekiwi_description/urdf/lekiwi.urdf.xacro` |

## 안전 처리 계층

| 상황 | 처리하는 곳 | 결과 |
|------|-------------|------|
| 드라이버 시작 (바퀴 쓰기 켬) | 모터 드라이버 | 다른 프로그램이 남긴 바퀴 토크 · 속도 정리: 속도 0 → 토크 끔 |
| 드라이버 시작 (팔 쓰기 켬) | 모터 드라이버 | 팔 토크 · 목표는 건드리지 않는다. `arm_torque` true 전까지 팔 명령을 버린다 |
| 팔 명령에 팔이 아닌 ID · 같은 ID 두 번 · 길이 다름, 또는 이번 주기 위치를 읽지 못한 팔 | 모터 드라이버 | 메시지 전체를 버린다 |
| 팔 목표가 0~4095 밖이거나 그 모터의 마지막 위치에서 `arm_max_goal_offset_ticks`(1024) 넘게 떨어짐 | 모터 드라이버 | 그 안으로 자른다 (경고 로그). 잘못된 발행자가 팔을 크게 움직이지 못한다 |
| 팔 토크 켜기가 일부 팔에서 두 번 실패 | 모터 드라이버 | 호출 전에 꺼져 있던 팔만 다시 끈다. 잡고 있던 팔은 끄지 않는다. `arm_ready=false` |
| 팔 토크 끄기 응답이 없음 | 모터 드라이버 | 그 ID에만 1초마다 다시 끈다. 확인될 때까지 켜기를 거부한다 |
| bringup을 웹에서 끔 | 게이트웨이 → 팔로워 팔 | `lekiwi/arm/rest` → `rested` → `lekiwi/motor/arm_torque` false. 팔을 펴 둔 채 토크를 끄지 않는다 |
| 팔 명령이 끊김 | 모터 드라이버 | 아무것도 하지 않는다. 모터는 마지막 목표를 유지한다 (팔 · 물건이 떨어지지 않게) |
| 팔이 위치 모드가 아니거나 상태 비트가 켜짐 | 모터 드라이버 | 팔 토크를 켜지 않는다 |
| `cmd_vel`이 0.3 s 넘게 끊김 | 베이스 컨트롤러 | 바퀴 명령 0 |
| 엔코더(`lekiwi/motor/states`)가 0.5 s 넘게 끊기거나 바퀴가 응답하지 않음 | 베이스 컨트롤러 | 속도 0 → 토크 끔 → 고장 고정. `lekiwi/motor/power false` 후에만 다시 켤 수 있다 |
| 바퀴 명령이 0.5 s 넘게 끊김 (베이스 컨트롤러가 멈춤) | 모터 드라이버 | 속도 0 |
| 바퀴 명령이 3 s 넘게 끊김 | 모터 드라이버 | 바퀴 토크 끔 |
| 토크가 켜진 바퀴가 응답하지 않음 | 모터 드라이버 | 모든 바퀴 속도 0 → 토크 끔 |
| 드라이버가 바퀴 토크를 끔 (위 이유 등) | 베이스 컨트롤러 | `states`에서 `torque_enabled=false`를 보면 고장 고정, `ready=false` |
| 바퀴가 속도 모드가 아니거나 상태 비트(과열 · 과부하 · 과전류 · 전압)가 켜짐 | 모터 드라이버 | 토크를 켜지 않는다 |
| 드라이버 종료 | 모터 드라이버 | 바퀴 토크가 켜져 있었으면 속도 0 → 토크 끔. 팔 토크는 그대로 둔다 (마지막 목표 유지) |
| Nav2 주행 중 조작 화면(PC RViz 노드 `rviz` · `rviz2`)이 모두 사라짐 | Nav2 감독 도구 | 토크 끔 (`lekiwi/motor/power false`). 화면이 없으면 처음부터 켜지 않는다 |
| PC ZMQ 명령이 0.5 s 끊김 | ZMQ 호스트 | `cmd_vel` 0 한 번 (베이스 컨트롤러는 0.3 s에 이미 선다). 팔은 그 자리 |
| PC ZMQ 명령이 2 s 끊김 | ZMQ 호스트 | 세션 끝: `cmd_vel` 0 → 발행자 지움 → `lekiwi/motor/power` false. 팔 토크는 그대로 (팔이 떨어지지 않게) |
| ZMQ 세션 시작 때 웹 조작 화면(`lekiwi_web_operator_view`) · 다른 `cmd_vel` 발행자 · 비상 정지 · 팔로워 · 드라이버 · 베이스 파라미터 없음 | ZMQ 호스트 | 시작 거절 (`blocked`, 이유는 `lekiwi/zmq_host/status`). 토크를 켜지 않는다 |
| ZMQ 세션 중 바퀴 · 팔 토크가 밖에서 꺼짐 (비상 정지) | ZMQ 호스트 | `blocked`, 바로 `cmd_vel` 발행자를 지운다 (남기면 게이트웨이가 비상 정지를 못 푼다). 명령이 2 s 끊기기 전에는 토크를 다시 켜지 않는다 |
