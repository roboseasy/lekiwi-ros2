# lekiwi_zmq_host

lerobot LeKiwi 호스트(`lekiwi_host`)와 **같은 ZMQ 프로토콜**을 ROS 토픽 · 서비스로 바꾸는 ROS 2 노드 패키지.
PC(Physical Labs, lerobot 예제 스크립트)는 ROS 없이 lerobot `LeKiwiClient` 로 붙어 관측을 받고 명령을 보낸다.
이 노드는 **번역만** 한다: 팔 보정 · 램프 · 튐 막기는 팔로워 팔(`lekiwi_lerobot`), 바퀴 상한 · 끊김은 베이스 컨트롤러가 그대로 맡는다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · **시스템 python** (`python3-zmq`). lerobot 을 import 하지 않는다
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_zmq_host`
- 상대 노드: 팔로워 팔 (`lekiwi/lerobot/state` · `lekiwi/arm/goal`), 베이스 컨트롤러 (`cmd_vel` · `lekiwi/motor/power` · 기구학 파라미터), 모터 드라이버 (`lekiwi/motor/arm_torque`), 카메라, 웹 게이트웨이 (`lekiwi/safety/estop_latched` 를 내고 `lekiwi/zmq_host/status` 를 읽는다)

> ⚠️ **로봇이 움직인다.** 명령이 오면 바퀴 토크를 켜고 `cmd_vel` 을 내며, 팔 명령이 오면 팔 토크를 켠다.
> 포트에는 **인증이 없다** (옛 호스트와 같다): 같은 망의 누구나 붙을 수 있다.

---

## 흐름

```
PC LeKiwiClient ──PUSH→ :5555 (action JSON) ──▶ lekiwi_zmq_host_node ──lekiwi/arm/goal (JointState)──▶ 팔로워 팔
                                                     │  바퀴: lerobot _body_to_wheel_raw → 베이스 기구학의 역 ──cmd_vel──▶ 베이스 컨트롤러
PC LeKiwiClient ◀─PULL─ :5556 (관측 JSON) ◀──────────┤◀── lekiwi/lerobot/state (팔로워) · 카메라 JPEG 2대 (그대로 base64)
                                                     └──▶ lekiwi/zmq_host/status (JSON) ──▶ 게이트웨이 /api/status
```

## 메시지 (ZMQ)

**관측** (`PUSH tcp://*:5556`, `CONFLATE=1`, 30 Hz 이하): JSON 하나.

```json
{"arm_shoulder_pan.pos": 0.0, "…": 0.0, "theta.vel": 0.0,
 "front": "<base64 JPEG>", "wrist": "<base64 JPEG>",
 "_lekiwi_ros": {"v": 1, "jpeg": "standard", "units": "degrees", "session": "idle", "reason": null,
                 "follower_state": "following", "state_age_ms": 12, "front_age_ms": 20, "wrist_age_ms": null}}
```

- 상태 9개는 `lekiwi/lerobot/state` 의 `names` · `values` 그대로. 이름 · 순서가 lerobot LeKiwi 와 다르면 쓰지 않는다.
- **상태가 0.5 s 넘게 없으면 관측을 보내지 않는다.** `LeKiwiClient` 는 빠진 키를 0.0 으로 채우므로, 보내면 0 자세가 녹화 · 추론으로 나간다. 이유는 `lekiwi/zmq_host/status` 의 `reason` 에 (예: 팔로워의 "보정 파일이 없습니다").
- 카메라는 1 s 안의 프레임만 (없으면 키가 없고 나이는 null). `CompressedImage.data` 를 **다시 압축하지 않는다** (Pi CPU). 정상 JPEG 라서 바닐라 `LeKiwiClient` 는 색 순서가 뒤집혀 보인다 (옛 호스트는 RGB 배열을 BGR 로 압축했다) — Physical Labs 가 맞춘다.
- 카메라 구독은 관측 소켓에 클라이언트가 붙어 있을 때만 둔다 (socket monitor 로 접속 수를 센다).

**명령** (`PULL tcp://*:5555`, `CONFLATE=1`): lerobot action dict 의 JSON.

- `arm_*.pos` 6개는 모두 있거나 모두 없거나. 일부면 팔 부분만 버리고 `reason`. 팔 키가 없으면 팔은 그대로.
- `x.vel` · `y.vel` [m/s] · `theta.vel` [deg/s] (lerobot 몸체 기준) 은 없으면 0.
- 아는 키의 값이 유한한 수가 아니거나 절댓값이 1e4 보다 크면 명령 전체를 버린다 (명령이 온 것으로 치지 않는다). 모르는 키는 무시.

## 바퀴 변환

lerobot `LeKiwi._body_to_wheel_raw` 와 같은 식(바퀴 0.05 · 몸체 0.125 · 각 `[240, 0, 120] − 90` · `max_raw` 3000 비례 축소 · 모터 7/8/9)으로
바퀴 속도를 구한 뒤, 베이스 컨트롤러 기구학(`omni_kinematics.cpp`)의 역으로 그 바퀴 속도를 내는 `cmd_vel` 을 구한다.
기구학 값(`wheel_radius` · `base_radius` · `kinematics_frame_x/y/yaw` · `wheel_ids` · `wheel_directions`)은 `lekiwi_base_controller_node` 의
파라미터 서비스로 읽는다 (1 Hz, 컨트롤러가 사라지면 잊고 다시 읽는다). **못 읽으면 세션 시작을 거절한다** (기본값으로 돌지 않는다).

- 순수 x · y 는 1:1, 회전 90 °/s 는 `wz` 0.916 × π/2 rad/s 에 (−0.0155, −0.0109) m/s 가 섞인다 (기구학 원점이 다르다).
- 그 뒤 베이스 컨트롤러가 축마다 0.23 m/s · 1.53 rad/s 상한을 건다: 상한 안에서는 옛 호스트와 같게 돈다.

## 세션

| 상태 | 들어가는 때 | 하는 일 |
|------|------|------|
| `idle` | 시작 · 세션 끝 | 관측만 보낸다 |
| `waiting` | idle 에서 첫 유효 명령, 시작 거절 있음 | 명령을 버린다 (토크 · `cmd_vel` 발행자를 건드리지 않는다). 명령이 오는 동안 **1 s 마다 시작 거절을 다시 보고**, 풀리면 그 명령으로 active. 명령이 2 s 끊기면 idle |
| `active` | idle · waiting 에서 유효 명령, 시작 거절 없음 | `lekiwi/motor/power` true, `cmd_vel` 발행자 만듦. 첫 팔 명령 때 `lekiwi/motor/arm_torque` true. 명령마다 `lekiwi/arm/goal` · `cmd_vel` |
| `blocked` | 안전 막힘: active 중 바깥에서 토크 꺼짐 · 비상 정지 · 토크 켜기 실패 | 명령을 버린다. **명령이 2 s 끊겨야 idle** (계속 보내는 클라이언트가 비상 정지를 저절로 풀지 못하게) |

- 시작 거절 (`reason`): 웹 조작 화면 노드 `lekiwi_web_operator_view` 가 있음 · 다른 `cmd_vel` 발행자 (`collision_monitor` 면 "자율 주행 중") · `lekiwi/safety/estop_latched` true · 팔로워 · 드라이버 노드 없음 · 베이스 파라미터 없음.
  대개 일시적이다 (베이스 파라미터를 아직 못 읽음 · 팔로워가 뜨는 중 · 웹 권한을 막 놓아 조작 화면 노드가 그래프에 1 s 남짓 남음): 그래서 blocked 가 아니라 waiting 이고, 앱이 30 Hz 로 계속 보내도 풀리면 저절로 시작한다.
- **세션 주인**: 세션(waiting · active · blocked)을 연 명령의 ZMQ `Peer-Address` 가 주인 (`client_ip`). 다른 주소의 명령은 버리고 감시 시간도 늘리지 않으며,
  마지막으로 버린 뒤 3 s 동안 상태에 `rejected_peer` 와 `reason` "다른 PC(<주인 IP>)가 조작 중입니다" 를 싣는다 (세션 자체의 이유가 있으면 그것이 먼저). idle 이 되면 주인을 잊는다.
  pyzmq 가 주소를 주지 않으면 주인 검사 없이 돈다 (경고 로그 한 번).
- 명령이 0.5 s 끊기면 `cmd_vel` 0 한 번. 2 s 끊기면 세션 끝: `cmd_vel` 0 → 발행자 지움 → `power` false → idle. **팔 토크는 그대로** (팔이 떨어지지 않게, 웹 권한 끝과 같다).
- active 중 `lekiwi/motor/ready` 가 (세션 중 true 를 본 뒤) false, 또는 팔 토크를 켠 뒤 `arm_ready` 가 false → blocked, **바로 `cmd_vel` 발행자를 지운다** (게이트웨이는 다른 발행자가 있으면 비상 정지 풀기를 거절한다). `power` true 직후의 false 는 켜지는 중이라 보지 않는다.
- active 중 `lekiwi/safety/estop_latched` 가 true 가 되면 (바퀴 ready 를 보기 전이어도) blocked.
- 토크 켜기 서비스가 없거나 실패하면 blocked. 그 명령의 남은 일 (발행자 · 팔 토크 · 목표) 은 하지 않는다.
- active 에서 blocked 가 되면 언제나 이 세션이 켠 바퀴 토크를 바로 끈다 (`power false`, 팔 토크만 꺼졌을 때도).
- 노드가 꺼지면 `cmd_vel` 0 을 한 번 보낸다. 바퀴 토크는 드라이버가 바퀴 명령이 3 s 끊기면 끈다.

## 구독 (Subscribe)

| 토픽 | 타입 | QoS | 쓰임 |
|------|------|------|------|
| `lekiwi/lerobot/state` | `lekiwi_msgs/LerobotState` | reliable, 깊이 10 (팔로워와 같음) | 관측 상태 |
| `lekiwi/sensor/{front,wrist}_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | SensorData | 관측 카메라 (클라이언트가 있을 때만) |
| `lekiwi/arm/status` | `std_msgs/String` | 깊이 10 | `follower_state` · 상태가 없을 때 이유 |
| `lekiwi/motor/ready` · `lekiwi/motor/arm_ready` | `std_msgs/Bool` | reliable + transient_local | 바깥에서 토크가 꺼졌는지 |
| `lekiwi/safety/estop_latched` | `std_msgs/Bool` | reliable + transient_local | 걸려 있으면 시작 거절. 발행자가 없으면 '모름' (거절하지 않고 `estop_unknown: true`) |

## 발행 (Publish)

| 토픽 | 타입 | QoS | 내용 |
|------|------|------|------|
| `lekiwi/arm/goal` | `sensor_msgs/JointState` | SensorData (팔로워 구독과 같음) | 이름 `arm_shoulder_pan` … `arm_gripper`, LeRobot 단위 |
| `cmd_vel` | `geometry_msgs/Twist` | 깊이 10 | **세션이 active 일 때만 발행자가 있다** |
| `lekiwi/zmq_host/status` | `std_msgs/String` | reliable + transient_local | 바뀔 때 + 1 Hz: `{"session": "idle\|waiting\|active\|blocked", "reason": str\|null, "rejected_peer": str\|null, "clients": int, "client_ip": str\|null, "estop_unknown": bool}`. `client_ip` 는 세션 주인 (세션을 연 명령의 보낸 주소, idle 이면 null), `rejected_peer` 는 3 s 안에 버린 다른 주소 |

서비스 클라이언트: `lekiwi/motor/power` · `lekiwi/motor/arm_torque` (`std_srvs/SetBool`, 비동기), 파라미터: `lekiwi_base_controller_node/get_parameters`.

## 파라미터

| 이름 | 기본 | 뜻 |
|------|------|------|
| `port_zmq_cmd` | 5555 | 명령 포트 (bringup `zmq_cmd_port`) |
| `port_zmq_observations` | 5556 | 관측 포트 (bringup `zmq_obs_port`) |
| `max_loop_freq_hz` | 30.0 | 명령 읽기 · 관측 주기 |

시간 상수 (계약이라 파라미터로 두지 않는다): 상태 0.5 s · 카메라 1 s · 명령 0 0.5 s · 세션 끝 2 s · waiting 다시 보기 1 s · 버린 주소 표시 3 s.

## 빌드 · 시험

```bash
# 워크스페이스 루트에서. ydlidar 가 없는 PC 는 base-paths 로 다른 패키지를 빼고 빌드한다
colcon build --symlink-install --packages-select lekiwi_msgs lekiwi_zmq_host lekiwi_bringup \
  --base-paths src/lekiwi_msgs src/lekiwi_zmq_host src/lekiwi_bringup

# 순수 시험 (ROS 없이): 기구학 · 명령 해석 · 관측 조립 · 세션. lerobot 이 있으면 원본 _body_to_wheel_raw 와 대조
~/miniconda3/envs/physical-labs/bin/python -m pytest -q src/lekiwi_zmq_host/test

# ROS 통합 시험까지: 가짜 팔로워 · 드라이버 · 베이스 컨트롤러 · 카메라 · 게이트웨이 + 바닐라 lerobot LeKiwiClient.
# ROS 의 pytest 플러그인(launch_testing)은 conda python 에서 import 되지 않으니 자동 로드를 끈다
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ~/miniconda3/envs/physical-labs/bin/python -m pytest -q src/lekiwi_zmq_host/test

# 시스템 python (Pi 와 같은 환경): lerobot · zmq 가 없는 시험은 건너뛴다
ROS_DOMAIN_ID=185 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --packages-select lekiwi_zmq_host
```

통합 시험은 도메인 180~189 · `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로 띄워 다른 ROS 그래프와 섞이지 않는다.
확인하는 것: 관측 9개 · 카메라 2대 수신, action → `lekiwi/arm/goal` · `cmd_vel` · 토크 서비스, 2 s 끊김 → 세션 끝 (`power` false, 팔 토크 그대로),
웹 조작 화면 노드가 있으면 waiting · 그 노드가 사라지면 계속 보내던 클라이언트가 active, 상태 발행을 멈추면 관측이 멈춤.

## 실행

```bash
# bringup 이 기본으로 켠다 (use_zmq_host:=true). 끄기 / 포트 바꾸기
ros2 launch lekiwi_bringup bringup.launch.py use_zmq_host:=false
ros2 launch lekiwi_bringup bringup.launch.py zmq_cmd_port:=5555 zmq_obs_port:=5556

# 따로
ros2 run lekiwi_zmq_host lekiwi_zmq_host_node --ros-args \
  --params-file ~/lekiwi_ws/install/lekiwi_zmq_host/share/lekiwi_zmq_host/config/lekiwi_zmq_host.yaml

# 세션 상태
ros2 topic echo lekiwi/zmq_host/status
```

PC (lerobot 0.6.0):

```python
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
robot = LeKiwiClient(LeKiwiClientConfig(remote_ip='<로봇 IP>', id='lekiwi'))
robot.connect()   # 상태가 없으면 (보정 없음 등) 첫 관측을 못 받아 5 s 뒤 실패한다 — status 의 reason 을 본다
```

## 한계

- 인증 없음 (옛 호스트와 같다). 교실 배포 전 로봇 쪽에서 다룬다.
- 다른 조작 주체와의 배타는 ROS 그래프 (조작 화면 노드 · `cmd_vel` 발행자) 와 상태 토픽에 기댄다. 판단이 게이트웨이와 이 노드 두 곳이다.
- 시작 거절 검사는 세션을 시작할 때 (idle · waiting) 만 한다. active 중에는 토크 꺼짐만 본다 (웹 주행은 '다른 `cmd_vel` 발행자'로 게이트웨이가 막는다).
- 팔로워 안전 제한(45° 튐 막기 · 2 s 램프)이 정책 · EEF 명령에도 걸린다.
