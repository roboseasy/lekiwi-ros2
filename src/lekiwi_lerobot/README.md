# lekiwi_lerobot

lerobot 0.6.0의 보정 · 단위로 **LeKiwi 팔로워 팔을 움직이는 ROS 2 노드** 패키지.
리더 관절 목표(LeRobot 단위)를 받아 시작 램프 · 범위 제한 · 튐 막기를 거쳐 모터 드라이버에 tick 명령으로 보낸다. 따라가는 중에는 목표를 받자마자 그대로 보낸다.
PC에서 lerobot 데이터셋을 기록하도록 **lerobot LeKiwi `observation.state`와 같은 값**(팔 6개 + 몸체 속도 3개)도 낸다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · lerobot 0.6.0 (`~/lerobot060_venv`)
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_lerobot`
- 상대 노드: `lekiwi_motor_driver_node` (버스 주인. 이 노드는 **모터 버스를 열지 않는다**), 웹 오퍼레이터 게이트웨이 (목표를 보낸다)
- 메시지: `lekiwi_msgs/MotorStateArray`, `lekiwi_msgs/ArmCommand`, `lekiwi_msgs/ArmRegisters`, `lekiwi_msgs/LerobotState`, `sensor_msgs/JointState`, `std_msgs/Bool`, `std_msgs/String`

> ⚠️ **팔이 실제로 움직인다.** 드라이버가 팔 명령을 받는 동안(`lekiwi/motor/arm_ready` true, `lekiwi/motor/arm_torque` true를 부른 뒤) 목표가 오면 명령을 보낸다.
> 처음 시험할 때는 팔 주변을 비우고, 토크를 끌 사람(웹의 '팔 힘 빼기' · 비상 정지)을 둔다. 토크를 끄면 편 팔은 떨어진다.

---

## 흐름

```
게이트웨이 ──lekiwi/arm/goal (LeRobot 단위)──▶ lekiwi_follower_arm_node ──lekiwi/motor/arm_command (tick)──▶ 모터 드라이버
                                                  ▲          │
           lekiwi/motor/states (30 Hz) ───────────┤          ├─▶ lekiwi/arm/state (LeRobot 단위)
           lekiwi/motor/arm_ready ────────────────┤          ├─▶ lekiwi/arm/status (JSON)
           lekiwi/motor/arm_registers ────────────┘          └─▶ lekiwi/lerobot/state (lerobot observation.state 9개, PC 기록기)
                                     (모두 드라이버가 낸다)
게이트웨이 ──lekiwi/arm/rest (Trigger, bringup 끄기 전)──▶ lekiwi_follower_arm_node: 접은 자세로 천천히 → status rested → 게이트웨이가 팔 토크 끔
```

- 단위 · 보정은 이 노드에서만 다룬다. `arm_command`는 raw tick이다.
- tick ↔ 단위 변환은 lerobot 코드 그대로다: 포트를 열지 않은 `FeetechMotorsBus`(`use_degrees=True`, 그리퍼 `RANGE_0_100`)의 `_normalize` · `_unnormalize`.

### LeRobot 단위

| 관절 | 단위 | 0의 뜻 |
|------|------|------|
| `arm_shoulder_pan` · `arm_shoulder_lift` · `arm_elbow_flex` · `arm_wrist_flex` · `arm_wrist_roll` | 각도 [deg] | 보정 범위(`range_min`~`range_max`) 가운데. `(tick − 가운데) × 360 / 4095` |
| `arm_gripper` | 0~100 | `range_min`이 0, `range_max`가 100 |

lekiwi04 보정 범위: pan ±120.6°, lift ±106.1°, elbow ±97.2°, wrist_flex ±100.8°, roll ±180°, 그리퍼 0~100.

---

## 노드

- `lekiwi_follower_arm_node` (실행 파일 `lekiwi_follower_arm_node`, 모듈 `lekiwi_lerobot/lekiwi_follower_arm_node.py`)
- 판단은 ROS 없는 `follower_logic.FollowerArmLogic` (입력 검사) · `FollowerController` (램프 · 이동 제한 · 끊김 유지)가 한다. 노드는 ROS 입출력만 한다.

## 구독 (Subscribe)

| 토픽 | 타입 | QoS | 내용 |
|------|------|------|------|
| `lekiwi/motor/states` | `lekiwi_msgs/MotorStateArray` | SensorData | 드라이버 모터 상태 (30 Hz). 팔 6개의 `id` · `online` · `position_ticks` · `status`, 바퀴 3개의 `id` · `online` · `velocity`를 쓴다 (`torque_enabled`는 쓰지 않는다) |
| `lekiwi/motor/arm_ready` | `std_msgs/Bool` | reliable, transient_local, 깊이 1 | 드라이버가 팔 명령을 받는가. `lekiwi/motor/arm_torque` true가 성공한 뒤에만 true, 시작 · 토크 끔 · 켜기 실패 뒤 false. 받기 전에는 false로 본다 |
| `lekiwi/motor/arm_registers` | `lekiwi_msgs/ArmRegisters` | reliable, transient_local | 드라이버가 읽은 팔 모터 `homing_offset` · `min/max_position_limit` · P/I/D. 보정 파일과 맞춰 본다 (아래) |
| `lekiwi/arm/goal` | `sensor_msgs/JointState` | best effort, 깊이 5 (reliable · best effort 발행 모두 받는다) | 리더 관절. `name`에 `arm_*` 6개가 모두, `position`이 LeRobot 단위의 유한한 값이어야 받는다. 하나라도 빠지면 메시지를 버린다. 접은 자세 모드(`lekiwi/arm/rest`)에서는 무시한다 |

## 발행 (Publish)

| 토픽 | 타입 | QoS | 내용 |
|------|------|------|------|
| `lekiwi/arm/state` | `sensor_msgs/JointState` | reliable, 깊이 10 | 팔로워 지금 관절 (LeRobot 단위). 모터 상태가 올 때마다 (30 Hz). stamp는 모터 상태의 stamp. 팔 모터 하나라도 응답이 없으면 내지 않는다 |
| `lekiwi/lerobot/state` | `lekiwi_msgs/LerobotState` | reliable, 깊이 10 | lerobot LeKiwi `observation.state` 9개 (아래 'lerobot 관측 상태'). 모터 상태가 올 때마다 (30 Hz). stamp는 모터 상태의 stamp. 팔 6개 · 바퀴 3개 중 하나라도 그 샘플에서 응답이 없거나, 팔 · 바퀴 모터 ID가 lerobot LeKiwi와 다르거나, `arm_registers`가 아직 없거나 보정 파일과 다르면 내지 않는다 |
| `lekiwi/motor/arm_command` | `lekiwi_msgs/ArmCommand` | reliable, 깊이 1 | 팔 목표 tick. `ids` [1..6], `positions_ticks` 0~4095. 따라가는 중: 목표를 받을 때마다 바로 (지난 명령 tick ± `follow_step_deg` 안). 램프 · 접은 자세 · 멈춘 뒤 첫 명령: 잰 tick ± `max_step_deg` 안, `rate_hz` 타이머로 (지난 한 주기 안에 목표로 명령을 냈으면 건너뛴다: 같은 명령을 두 번 내지 않는다) |
| `lekiwi/arm/status` | `std_msgs/String` (JSON) | reliable, 깊이 10 | 상태. `rate_hz`로 늘 낸다 (아래) |

## 서비스

| 서비스 | 타입 | 동작 |
|------|------|------|
| `lekiwi/arm/rest` | `std_srvs/Trigger` | 접은 자세로 천천히 돌아간다 (아래 '접은 자세'). 게이트웨이가 bringup을 끄기 전에 부르고, `lekiwi/arm/status`가 `rested`가 되면 `lekiwi/motor/arm_torque` false를 부른다. 이 노드는 토크를 끄지 않는다 |

### 접은 자세 (`lekiwi/arm/rest`)

- **거절** (`success: false`, `message`에 한국어 이유. 아무것도 바꾸지 않는다): 보정 · lerobot을 불러오지 못했다 (`팔로워를 쓸 수 없습니다: …`),
  `arm_ready` false (`팔 토크가 꺼져 있습니다`), 명령할 수 없다 (모터 상태 끊김, 걸어 둔 모터 이상, `arm_registers` 없음 · 보정 파일과 다름: 그 오류 글),
  `rest_pose_ticks`가 비었다 (`접은 자세가 설정되지 않았습니다 (rest_pose_ticks)`), 이번 모터 상태에서 팔 모터가 응답하지 않았다.
- **성공하면 접은 자세 모드** (`message`: `접은 자세로 돌아갑니다`): 리더 목표(`lekiwi/arm/goal`)를 무시하고,
  잰 자세 → 접은 자세로 `rest_time_s`(3 s) 동안 smoothstep 램프 (시작 램프와 같은 모양). 한 주기 이동 제한(`max_step_deg`, tick ± 114)과 보정 범위 자르기는 그대로다.
  모든 관절이 이미 `rest_tolerance_deg` 안이면 램프 없이 바로 `rested` (`이미 접은 자세입니다`). 다시 부르면 그때 잰 자세에서 다시 시작한다.
- **접은 자세는 보정 범위로 자른다**: lekiwi04 접은 팔꿈치(tick 3190, 105.7°)는 보정 범위(3094, 97.2°) 밖이라, 편 팔에서 부르면 **범위 끝 3094에서 멈춘다** (약 8° 덜 접힌다).
  그 뒤 토크를 끄면 팔꿈치는 제 무게로 처진다. 그리퍼처럼 lerobot이 0~100으로 잘라 읽는 관절도 범위 끝까지만 간다.
  **이미 그쪽 범위 밖에 있는 관절은 그 자리에 둔다** (실물로 접혀 있는 팔꿈치 3190): 범위 끝까지 8° 펴지 않고 다 온 것으로 본다.
- **부르는 쪽은 먼저 리더 목표를 멈춘다.** 접은 자세 중에 명령을 막는 이상이 생기면 (걸어 두지 않는 것도: 모터 상태 끊김 · 팔 모터 응답 없음) 접은 자세 모드에서 나오고
  다시 들어가지 않는다. 이상이 풀린 뒤 리더 목표가 아직 오고 있으면 그 목표를 따라간다.
- **상태**: 가는 동안 `resting` (`ramp_progress` 0~1, `diffs` = 자른 접은 자세 − 팔로워). 모든 관절이 자른 접은 자세에서 `rest_tolerance_deg`(3°, 그리퍼는 같은 모터 각도) 안이면 `rested`.
  램프가 끝나고 1초가 지나도 못 온 관절이 있으면 그래도 `rested`로 하고 `warnings`에 `접은 자세에 다 오지 못했습니다: <관절들>`을 적는다 (게이트웨이가 토크를 끌 수 있게).
- **`rested`인 동안**: 그 주기에 마지막 명령을 한 번 보내고 그친다. 드라이버가 그 목표를 유지한다 (계속 보내지 않는다).
- **나오기**: `arm_ready` 메시지가 오면 (토크 끔, 같은 값이 다시 와도) 또는 명령을 막는 이상(모터 상태 끊김 · 모터 이상 · 응답 없음 · 보정 다름)이 생기면,
  지금까지의 멈춤 규칙과 같이 그친다. 다음 리더 목표는 잰 자세에서 다시 출발한다.
- **접은 자세는 로봇마다 다르다**: 패키지 기본은 없음(`[]`, 요청을 거절한다). 그 로봇의 장비 프로파일 `~/lekiwi_ws/robot_profile.yaml`에 둔다
  (`lekiwi_bringup`이 패키지 설정 위에 얹는다, `lekiwi_bringup/config/robot_profile.yaml`의 예). 웹 게이트웨이 보정 탭이 쓰는 `~/lekiwi_ws/arm_profile.yaml`이
  있으면 그 값(`robot_id` · `rest_pose_ticks`)이 장비 프로파일을 덮는다 (`lekiwi_bringup` README '파라미터 순서'). 팔을 접고 `lekiwi/motor/states`의 팔 `position_ticks`(ID 1~6)를 적는다.
  예: lekiwi04 (2026-10-22 가짜 버스 시험에 쓴 값, **4단계 실물 시험에서 다시 잰다**)

  ```yaml
  lekiwi_follower_arm_node:
    ros__parameters:
      rest_pose_ticks: [1787, 1230, 3190, 2636, 3235, 2002]
  ```

### lerobot 관측 상태 (`lekiwi/lerobot/state`)

PC에서 lerobot `LeRobotDataset`(v3)으로 기록해 **lerobot LeKiwi 스택이 기록한 데이터셋과 합칠 수 있게** 같은 뜻으로 낸다
(`robot_type` `lekiwi_client`, 30 fps, `observation.state` float32[9]).

| `names` (순서 고정) | 단위 | 계산 |
|------|------|------|
| `arm_shoulder_pan.pos` … `arm_wrist_roll.pos` | [deg] | `lekiwi/arm/state`와 같은 값 (lerobot `_normalize`, `use_degrees=True`) |
| `arm_gripper.pos` | 0~100 | 같음 (`RANGE_0_100`) |
| `x.vel` · `y.vel` | [m/s] | 바퀴 `Present_Velocity`로 lerobot `LeKiwi._wheel_raw_to_body` (`follower_logic.wheel_raw_to_body`, 같은 상수 · 순서 · numpy 계산) |
| `theta.vel` | [deg/s] | 같음 |

- 바퀴: lerobot `base_left_wheel` · `base_back_wheel` · `base_right_wheel` = ID 7 · 8 · 9 (실물도 왼쪽 · 뒤 · 오른쪽, `lekiwi_description/docs/FRAME_MIGRATION.md`).
  드라이버 `velocity` [rad/s]는 `Present_Velocity` tick × 2π / 4096 이라 반올림해 정수 tick으로 되돌린 뒤 lerobot 식에 넣는다.
  부호는 드라이버 · lerobot 모두 부호-크기 bit 15 (`Present_Velocity`, lerobot `STS_SMS_SERIES_ENCODINGS_TABLE`)이고, lerobot은 이 값을 정규화하지 않는다.
- **ROS `odom`을 쓰지 않는다**: 베이스 컨트롤러는 몸체 반지름 0.13647 m (lerobot 0.125 m, 회전이 8.4% 작게 나온다), rad/s, 위치 차분 0.3 s 창이다.
- 팔 · 바퀴 ID가 lerobot과 다르거나 속도가 유한하지 않아도 내지 않는다. 팔 보정 확인(`arm_registers`)이 끝나기 전에는 내지 않는다: 다른 모터 · 다른 로봇의 보정으로 계산한 각도가 데이터셋에 섞이지 않게.
- `action`(리더 목표 + 몸체 속도 명령)은 이 노드가 내지 않는다. PC 기록기가 보낸 목표로 만든다.

**메시지를 새로 만든 이유** (`lekiwi_msgs/LerobotState`: `std_msgs/Header header`, `string[] names`, `float64[] values`):
`sensor_msgs/JointState`의 `position`에 몸체 속도(`x.vel`)를 넣으면 뜻이 틀린다. `std_msgs/Float64MultiArray`에는 stamp(`header`)가 없다.
`names`를 함께 실어 기록기가 lerobot 특징 이름으로 바로 맞추고 순서가 바뀌면 알아챌 수 있다. 값은 계산한 float64 그대로 싣고, 데이터셋에 쓸 때 float32로 바꾼다.

### `lekiwi/arm/status` JSON

```json
{
  "state": "ramping",
  "reason": "시작 자세 차이가 커서 2초 동안 맞춥니다: arm_elbow_flex",
  "robot_id": "lekiwi04",
  "diffs": {"arm_shoulder_pan": 1.2, "arm_shoulder_lift": -0.4, "arm_elbow_flex": -105.67,
            "arm_wrist_flex": 0.0, "arm_wrist_roll": 0.3, "arm_gripper": 48.02},
  "ramp_progress": 0.62,
  "warnings": []
}
```

| 키 | 타입 | 뜻 |
|------|------|------|
| `state` | 문자열 | `idle` (명령하지 않음, 정상) · `ramping` (시작 램프) · `following` (따라가는 중) · `resting` (접은 자세로 가는 중) · `rested` (접은 자세에 왔다) · `error` (명령할 수 없음) |
| `reason` | 문자열 (한국어) | 사람이 읽는 이유. `following`일 때는 빈 문자열 |
| `robot_id` | 문자열 | 쓰는 보정 파일 이름 (`<robot_id>.json`) |
| `diffs` | 객체 | 관절 → (보정 범위로 자른 목표 − 팔로워) [LeRobot 단위], 소수 2자리. 명령하는 동안(`ramping` · `following`)과 접은 자세(`resting` · `rested`, 목표 = 자른 접은 자세)에 채우고 나머지는 `{}` |
| `ramp_progress` | 수 또는 `null` | 램프 진행 0~1 (소수 3자리). `ramping`이 아니면 `null` |
| `warnings` | 문자열 목록 (한국어) | 명령을 막지 않는 경고 (아래 표). 없으면 `[]` |

| `state` | `reason` 예 | 언제 |
|------|------|------|
| `error` | `보정 파일이 없습니다: <경로>` | 보정 파일이 없다. 보정한 뒤 노드를 다시 켠다 |
| `error` | `보정을 불러오지 못했습니다 (<경로>): <오류>` | lerobot이 없는 python으로 켰거나 (`No module named 'draccus'`), lerobot이 0.6.x가 아니거나 (`lerobot 0.7.0은 쓸 수 없습니다 (0.6.x만: …)`), 팔 관절이 빠졌거나, 파일의 모터 ID가 1~6이 아니다. 노드를 다시 켜야 한다 |
| `error` | `모터 상태가 0.5초 넘게 오지 않습니다 (lekiwi_motor_driver_node를 켜세요)` | 모터 상태가 `state_timeout_s` 넘게 끊겼다 (노드를 막 켰을 때도) |
| `error` | `팔 모터 ID가 다릅니다: arm_elbow_flex 4 (LeKiwi 3)` | 모터 상태의 이름 · ID가 lerobot LeKiwi(1~6)와 다르다 (드라이버 설정 `motor_ids` · `motor_names`) |
| `error` | `팔 모터 이상으로 명령을 멈춥니다: arm_elbow_flex 과부하 (이상이 풀린 뒤 원격 조작을 다시 시작하면 이어 갑니다)` | 팔이 켜져 있을 때 응답한 팔 모터의 `status`에 이상 비트(전압 · 각도 센서 · 과열 · 과전류 · 과부하)가 섰다. 그리퍼 과부하는 오류가 아니라 경고다 (물건을 쥐면 생기고 모터가 스스로 힘을 줄인다). 걸린 주기에 잰 자세를 한 번 보내 막힌 관절이 계속 밀지 않게 하고, 걸어 둔다. 비트가 내려가고 목표가 `goal_timeout_s` 넘게 끊기면 (사람이 원격 조작을 멈췄다) 또는 `arm_ready`가 새로 오면 풀리고, 다음 목표는 잰 자세에서 출발한다. 팔 토크를 끄지 않아도 된다 |
| `error` | `팔 모터가 응답하지 않습니다: arm_gripper` | 팔 모터 하나라도 `online` false가 `state_timeout_s` 넘게 이어졌다. 그보다 짧으면 그 주기만 쉰다 (경고) |
| `idle` | `드라이버의 팔 모터 보정 값을 기다립니다 (lekiwi/motor/arm_registers)` | 아직 `arm_registers`를 받지 못했다 |
| `error` | `보정 파일이 이 로봇의 모터와 다릅니다 (lekiwi04.json): arm_gripper homing_offset 모터 -470 · 파일 -473` | 모터의 `homing_offset` · `min_position_limit` · `max_position_limit`가 보정 파일의 `homing_offset` · `range_min` · `range_max`와 다르거나, 드라이버가 읽지 못한 관절이 있다 (`… arm_gripper 읽지 못함`). lerobot `is_calibrated`와 같은 기준. 다른 로봇의 파일이거나 보정 뒤 모터 설정이 바뀌었다 |
| `idle` | `팔 토크가 꺼져 있습니다` | `arm_ready` false (팔 토크가 꺼져 있거나, 이 드라이버 프로세스에서 아직 켜지 않았다) |
| `idle` | `목표를 기다립니다` | 아직 목표가 오지 않았다 |
| `idle` | `목표가 0.5초 넘게 오지 않아 자세를 유지합니다` | 목표가 `goal_timeout_s` 넘게 끊겼다 |
| `ramping` | `시작 자세 차이가 커서 2초 동안 맞춥니다: <관절들>` | 멈춰 있다가 목표가 왔고, 차이가 큰 관절이 있다 |
| `following` | (빈 문자열) | 따라가는 중 |
| `resting` | `접은 자세로 3초 동안 돌아갑니다` | `lekiwi/arm/rest`가 성공했다. 리더 목표를 무시한다 |
| `rested` | `접은 자세입니다` | 모든 관절이 접은 자세에 왔다 (또는 램프 뒤 1초, `warnings`). 팔 토크를 꺼도 된다 |

| `warnings` 예 | 언제 |
|------|------|
| `접은 자세에 다 오지 못했습니다: arm_wrist_flex` | `rested`인데 그 관절이 자른 접은 자세에서 `rest_tolerance_deg` 넘게 떨어져 있다 (램프가 끝나고 1초 기다렸다) |
| `팔이 명령을 따라오지 않습니다: arm_shoulder_pan (막혔거나 힘이 모자랍니다)` | 몸 관절이 명령과 `track_tolerance_deg` 넘게 떨어진 채 `track_timeout_s` 동안 그만큼도 움직이지 않았다. 명령은 그대로 낸다 (잰 자세 ± `max_step_deg` 안이라 쌓이지 않는다) |
| `팔 모터 응답이 잠깐 끊겨 이번 주기는 쉽니다: arm_elbow_flex` | 팔 모터 응답이 `state_timeout_s`보다 짧게 끊겼다. 명령 · 램프는 이어 간다 |
| `팔 모터 PID가 lerobot LeKiwi(P 16 · I 0 · D 32)와 다릅니다: arm_gripper P 32 · I 0 · D 32` | `arm_registers`의 P/I/D가 lerobot LeKiwi가 쓰는 값과 다르다. 움직임은 막지 않는다 |

---

## 안전 규칙

- **명령하지 않는 경우** (드라이버가 마지막 목표를 유지한다, 위가 먼저): 보정 · lerobot을 불러오지 못했을 때, 모터 상태가 `state_timeout_s`(0.5 s) 넘게 끊겼을 때,
  팔 모터 ID가 다르거나 이상 비트가 섰을 때, 팔 모터 응답이 `state_timeout_s` 넘게 없을 때 (짧으면 그 주기만), `arm_registers`가 없거나 보정 파일과 다를 때,
  `arm_ready` false일 때, 목표가 없거나 `goal_timeout_s`(0.5 s) 넘게 끊겼을 때. 이 노드는 마지막 목표를 계속 보내지 않는다.
- **`arm_ready`로만 판단한다.** 드라이버는 이 드라이버 프로세스에서 `lekiwi/motor/arm_torque` true가 성공한 뒤에만 명령을 받고, 그동안만 `arm_ready`가 true다.
  `torque_enabled` true는 받는다는 뜻이 아니다 (드라이버 재시작, 다른 프로그램이 켜 둔 토크). `arm_ready`가 false → true가 되면 잰 자세에서 새로 출발한다 (램프 여부를 다시 본다).
  이미 켜져 있을 때 true를 다시 부르면 드라이버는 아무것도 쓰지 않고 `arm_ready`도 다시 내지 않는다 (팔을 그 자리에 세우지 않는다). 꺼져 있던 팔만 목표 = 지금 위치 → 토크를 켠다.
  드라이버는 `arm_ready`를 바뀔 때만 내므로, 이 노드는 `arm_ready`를 받을 때마다 잰 자세에서 새로 출발한다 (false → true가 이어 와 false를 놓쳐도).
  이 노드는 서비스를 부르지 않는다 (게이트웨이가 부른다).
- **보정 확인**: 시작할 때 읽은 보정 파일을 드라이버가 읽은 모터 레지스터(`arm_registers`)와 맞춘다 (lerobot `is_calibrated`와 같은 기준: `homing_offset`, `range_min` = `Min_Position_Limit`, `range_max` = `Max_Position_Limit`).
  다른 로봇의 보정 파일로 팔을 움직이지 않는다. P/I/D가 lerobot LeKiwi(16 · 0 · 32)와 다르면 경고만 한다.
- **시작 램프**: 명령을 그친 상태에서 목표가 오면, 팔로워와 차이가 `ramp_threshold_deg`(8°)를 **넘는** 관절이 하나라도 있을 때 `ramp_time_s`(2 s) 동안 팔로워 자세 → 지금 목표로 맞춘다.
  모양은 smoothstep(시작 · 끝 속도 0)이고, 램프 중 목표가 움직이면 그 목표를 따라간다. 차이가 작으면 바로 따라간다.
  토크가 꺼졌다 켜진 뒤 · 목표가 끊겼다 다시 온 뒤 · 팔이 따라오지 않아 그친 뒤 · 오류가 풀린 뒤에도 다시 본다 (그동안 팔을 손으로 옮겼어도 튀지 않는다).
- **따라가는 중 (`following`)**: 리더 목표를 **그대로** 보낸다 (lerobot 기본 `max_relative_target` None과 같다). 목표를 받자마자 명령을 낸다 (30 Hz 타이머를 기다리지 않는다).
  지난 명령에서 한 번에 `follow_step_deg`(45°) 넘게 튀는 값만 자른다 (튐 막기: 손목 ±180° 이음매, 깨진 값). 사람 손 속도는 자르지 않는다.
  tick으로도 지난 명령 tick ± round(`follow_step_deg` × 4095 / 360)(= 512 tick)로 한 번 더 자른다. 드라이버는 잰 위치 ± 1024 tick(90°)으로 자른다.
  그래서 팔이 막혀 있어도 명령은 목표까지 간다: 막힌 관절은 서보가 목표 쪽으로 계속 민다 (드라이버 ± 1024 안, `track_*` 경고).
- **램프 · 접은 자세 · 멈춘 뒤 첫 명령**: 명령은 **잰 자세**(가장 최근 `lekiwi/motor/states`)에서 `max_step_deg`(10°) 넘게 떨어지지 않는다
  (lerobot `ensure_safe_goal_position`과 같은 기준). tick으로도 잰 tick ± round(`max_step_deg` × 4095 / 360) (10° = 114 tick, 그리퍼 포함).
  멈춘 뒤 첫 명령 다음부터 (램프가 끝난 뒤부터) 따라가는 중 규칙이 된다.
- **따라가는 속도**: 2026-10-22 통합 시험 (실제 드라이버 30 Hz + 가짜 버스, 서보 1차 지연 60 ms · 최고 3000 tick/s ≈ 264°/s, pan 사인 리더).
  예전 규칙(따라가는 중에도 잰 자세 ± 10°, 드라이버 ± 200)은 팔을 리더보다 크게 늦게 했다 (실물 lekiwi04: 200~305 ms, 435°/s 리더에서 pan 오차 p95 70°).

  | 리더 | 규칙 | 지연 | 팔 최고 속도 | 팔 진폭 |
  |------|------|------|------|------|
  | 30° · 0.5 Hz (최고 94°/s) | 예전 (잰 자세 ± 10°) | 85~115 ms | 86~95°/s | 29.5° |
  | 30° · 0.5 Hz | 지금 (목표 그대로) | 75 ms | 96~98°/s | 29.5° |
  | 40° · 1 Hz (최고 251°/s) | 예전 | 200 ms | 103°/s | 21.7° (40° 중) |
  | 40° · 1 Hz | 지금 | 75 ms | 236°/s (가짜 서보 최고 264°/s) | 37.5° |

  남은 지연은 주로 서보(가짜 버스의 1차 지연 60 ms)와 모터 상태 주기다. 목표 → 버스 쓰기는 가운데 1 ms · p95 2 ms (예전 7~32 ms, 타이머를 기다렸다). 실물 서보 지연은 아직 재지 않았다.
- **따라오는지 감시 (경고만)**: 몸 관절 하나라도 지난 명령과 `track_tolerance_deg`(4°) 넘게 떨어진 채 `track_timeout_s`(0.3 s) 동안 그만큼도 움직이지 않으면
  `warnings`에 `팔이 명령을 따라오지 않습니다: …`를 적는다. 명령 · 상태는 바꾸지 않는다 (드라이버가 명령을 받는지는 `arm_ready`가 알려 준다).
  빨리 움직이며 뒤처지는 관절은 움직이고 있어서 걸리지 않는다. 그리퍼는 물건을 잡으면 막히는 것이 정상이라 보지 않는다.
  `track_tolerance_deg`는 `max_step_deg`보다 작아야 하고 (램프 중 막힌 관절 앞의 명령은 잰 자세 + `max_step_deg`에 머문다), 중력으로 처지는 정도보다는 커야 한다.
  따라가는 중에는 명령이 목표 그대로라 막힌 관절 앞의 차이가 더 크다 (그래도 움직이지 않으면 걸린다). 빠르게 움직이며 뒤처지는 관절은 움직이고 있어 걸리지 않는다.
- **그리퍼**: 각도 값(8° · 10°)은 모터 축 각도로 보고 0~100 단위로 바꿔 쓴다. lekiwi04는 1° ≈ 0.70 단위 (8° ≈ 5.6, 10° ≈ 7.0).
- **범위 제한**: 목표는 보정 범위(`range_min`~`range_max`)로 자른다. lerobot 변환은 몸 관절을 자르지 않는다 (999° → 13351 tick). tick을 0~4095로 한 번 더 자른다.
- **범위 밖에서 시작**: lekiwi04의 접은 자세는 팔꿈치가 105.7° (tick 3190)로 보정 범위(97.2°, 3094) **밖**이다.
  명령은 늘 잰 자세와 보정 범위 사이에 있으므로, 이때는 램프 처음 몇 주기만 범위 밖 값(3190 → 3094)을 보내고 한 번에 튀지 않는다.
  그리퍼는 lerobot이 0~100으로 잘라 읽으므로 범위 밖 그리퍼는 단위로는 차이가 0으로 보인다. 그래도 tick 이동 제한(114 tick/주기)으로 범위 끝까지 천천히 들어온다.
  ⚠️ 아직 실물로 보지 않았다: 실물 STS3215가 `Max_Position_Limit` 밖 목표를 한계로 자른다면, 토크를 켜는 순간 (목표 = 지금 위치 3190 → 3094) 팔꿈치가 약 8° 튈 수 있다.
  가짜 버스는 한계를 흉내 내지 않는다. 실물 시험 첫 단계에서 접은 자세로 토크를 켜 확인한다.
  `lekiwi/arm/rest`도 같은 까닭으로 팔꿈치를 3190이 아니라 범위 끝 3094까지만 접는다 (위 '접은 자세').

---

## 파라미터

기본값은 코드와 `config/follower_arm.yaml`이 같다. 실수 파라미터는 `30.0`처럼 소수점을 붙인다.

| 이름 | 타입 | 기본 | 뜻 |
|------|------|------|------|
| `robot_id` | string | 호스트 이름 (`''`이면 호스트 이름) | 보정 파일 `<calibration_dir>/<robot_id>.json` |
| `calibration_dir` | string | `~/.cache/huggingface/lerobot/calibration/robots/lekiwi` | lerobot 보정 폴더 |
| `rate_hz` | double | 30.0 | 명령 · 상태 주기 |
| `ramp_threshold_deg` | double | 8.0 | [deg] 시작 램프를 할 차이 |
| `ramp_time_s` | double | 2.0 | [s] 램프 시간 |
| `max_step_deg` | double | 10.0 | [deg] 램프 · 접은 자세 · 멈춘 뒤 첫 명령: 잰 자세에서 한 번 최대 이동 (tick 제한도 이 값) |
| `follow_step_deg` | double | 45.0 | [deg] 따라가는 중: 지난 명령에서 한 번 최대 이동 (튐 막기만, tick 제한도 이 값 = 512 tick). 사람 손보다 크고 이음매 튐(358°)보다 작게 |
| `goal_timeout_s` | double | 0.5 | [s] 목표가 끊기면 명령을 그친다 |
| `track_tolerance_deg` | double | 4.0 | [deg] 명령과 이만큼 넘게 떨어진 몸 관절을 감시한다. `max_step_deg`보다 작아야 한다 |
| `track_timeout_s` | double | 0.3 | [s] 그동안 그만큼도 움직이지 않으면 경고한다 |
| `state_timeout_s` | double | 0.5 | [s] 모터 상태가 끊기거나 팔 모터가 이만큼 응답하지 않으면 오류 |
| `rest_pose_ticks` | int[6] | `[]` (없음) | 접은 자세 [tick, 실물 값], 모터 ID 1~6 순서. 로봇마다 다르므로 그 로봇의 `~/lekiwi_ws/robot_profile.yaml`에 둔다 (아래 '접은 자세'). 보정으로 LeRobot 단위로 바꾸고 보정 범위로 자른다. `[]`이면 접은 자세 없음 (`lekiwi/arm/rest` 거절). 정수 6개 · 0~4095가 아니면 (타입이 틀려도) 오류 상태 `팔로워 파라미터가 잘못됐습니다: rest_pose_ticks는 …` |
| `rest_time_s` | double | 3.0 | [s] 접은 자세 램프 시간 |
| `rest_tolerance_deg` | double | 3.0 | [deg] 모든 관절이 이 안이면 `rested` (그리퍼는 같은 모터 각도). `max_step_deg`보다 작아야 한다 (다 왔을 때 마지막 명령이 접은 자세 그 자체가 되게) |
| `motor_states_topic` · `arm_ready_topic` · `arm_registers_topic` · `goal_topic` · `state_topic` · `command_topic` · `status_topic` · `lerobot_state_topic` | string | 위 표의 토픽 | 토픽 이름 |
| `rest_service` | string | `lekiwi/arm/rest` | 접은 자세 서비스 이름 |

---

## 파이썬 환경

lerobot 0.6.0은 numpy `>=2.0,<2.3`이 필요하고, ROS Jazzy의 시스템 python은 numpy 1.26이다. 그래서 이 노드만 **lerobot이 든 python으로** 실행한다.

- launch가 ROS 설정을 source한 셸에서 `prefix`로 `lerobot_python`(기본 `~/lerobot060_venv/bin/python`)을 붙인다.
  rclpy · 메시지는 ROS 경로(`PYTHONPATH`)에서, lerobot · numpy 2.2는 그 환경에서 온다 (2026-10-22 Pi에서 확인).
- 가상환경을 다시 만들지 않는다. lerobot은 rosdep 키가 없어서 `package.xml`에 넣지 않았다.
- 이 노드에 `cv_bridge`처럼 numpy 1.x로 빌드된 ROS 모듈을 넣지 않는다 (깨진다).
- lerobot import는 Pi에서 약 5.5초 걸린다. 노드가 시작할 때 한 번만 불러오고 걸린 시간을 로그에 남긴다 (`lerobot · 보정 … 불러옴: 5.5 s`).
- `_normalize` · `_unnormalize`는 lerobot 내부 함수다. 그래서 `lerobot.__version__`이 0.6.x가 아니면 오류 상태로 멈춘다.
  **lerobot을 올리면** 아래 단위 시험(공식과 비교 · 왕복 0 tick)을 먼저 돌리고 `follower_logic.SUPPORTED_LEROBOT`을 바꾼다.

---

## 빌드 · 시험

```bash
# 워크스페이스 루트에서 (lekiwi_msgs에 ArmCommand · ArmRegisters · LerobotState가 있어야 한다)
colcon build --symlink-install --packages-select lekiwi_msgs lekiwi_lerobot && source install/setup.bash

# 시스템 python: FollowerController · FollowerArmLogic · base_velocity · 접은 자세 시험이 돈다 (lerobot이 필요한 8개는 건너뛴다)
ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --packages-select lekiwi_lerobot --event-handlers console_direct+
colcon test-result --verbose

# lerobot python: 전부 (65개). 그 환경에 pytest가 있어야 한다. PC conda 환경은 ROS 경로를 빼고 돌린다
cd src/lekiwi_lerobot && ~/lerobot060_venv/bin/python -m pytest -q test
cd src/lekiwi_lerobot && env -u PYTHONPATH -u AMENT_PREFIX_PATH ~/miniconda3/envs/physical-labs/bin/python -m pytest -q test
```

시험은 ROS 노드를 만들지 않고 하드웨어에 접근하지 않는다. 보정은 `test/data/lekiwi04.json`(lekiwi04 사본)을 쓴다.
따라가는 중 규칙: 목표 그대로 · 지난 명령 ± `follow_step_deg` 튐 막기 (손목 이음매 포함) · 막힌 관절에도 목표 그대로 · 멈춘 뒤 첫 명령은 잰 자세 ± `max_step_deg`, tick 제한도 같은 규칙.
`FollowerArmLogic` 시험은 lerobot 공식을 그대로 쓴 `FakeConverter`로 돌고, lerobot이 있으면 `FakeConverter`가 `ArmConverter`와 같은지도 본다.
`base_velocity`는 lerobot 없이 정기구학 역변환 · 바퀴 이름 · ID · online · NaN 조건을 보고, lerobot이 있으면 무작위 바퀴 속도 2000개(± 3000, ± 32767)에서
lerobot `LeKiwi._wheel_raw_to_body`와 1e-9 안에서 같은지 본다 (드라이버 rad/s를 거친 값으로). 팔 확인(`is_arm_confirmed`)은 팔 ID와 레지스터가 모두 맞을 때만 참이다.
접은 자세 시험: 거절 (접은 자세 없음 · 토크 꺼짐 · 보정 다름 · 걸어 둔 모터 이상 · 상태 끊김 · 레지스터 없음), `rest_pose_ticks` 형식 (개수 · 범위 · 실수 · 문자열 · bool)과
`rest_tolerance_deg` < `max_step_deg`, smoothstep 모양, 잰 자세 기준 이동 제한, 범위 밖 팔꿈치 자르기 (편 팔에서) · 그 자리에 두기 (실물로 접혀 있을 때),
리더 목표 무시, 다 오면 `rested` · 그 뒤 명령 없음, 못 온 관절은 램프 뒤 1초에 경고와 함께 `rested`, `arm_ready` 메시지 · 모터 이상으로 나오기.

## 실행

```bash
# Pi (bringup이 드라이버를 arm_write_enabled로 켠 뒤)
ros2 launch lekiwi_lerobot follower_arm.launch.py
ros2 launch lekiwi_lerobot follower_arm.launch.py robot_id:=lekiwi04 lerobot_python:=/path/to/python
# launch 없이: ROS 설정을 source한 셸에서 lerobot python으로
~/lerobot060_venv/bin/python install/lekiwi_lerobot/lib/lekiwi_lerobot/lekiwi_follower_arm_node \
  --ros-args --params-file src/lekiwi_lerobot/config/follower_arm.yaml
```

| launch 인자 | 기본 | 뜻 |
|------|------|------|
| `params_file` | 이 패키지 `config/follower_arm.yaml` | 파라미터 파일 |
| `lerobot_python` | `~/lerobot060_venv/bin/python` | lerobot 0.6.0이 든 python (venv · conda) |
| `robot_id` | `''` | 보정 파일 이름. 비우면 `params_file`의 `robot_id` (그것도 비면 호스트 이름). 주면 문자열로 덮어쓴다 (`04`도 문자열) |

손으로 확인 (`arm_ready`가 true가 되기 전에는 명령을 보내지 않는다):

```bash
ros2 topic echo lekiwi/arm/status
ros2 topic echo lekiwi/arm/state
ros2 topic echo --qos-durability transient_local lekiwi/motor/arm_ready
ros2 topic echo --qos-durability transient_local lekiwi/motor/arm_registers
ros2 service call lekiwi/motor/arm_torque std_srvs/srv/SetBool '{data: true}'    # 팔이 움직일 수 있다
ros2 topic pub -r 30 lekiwi/arm/goal sensor_msgs/msg/JointState \
  "{name: [arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex, arm_wrist_flex, arm_wrist_roll, arm_gripper],
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]}"
```

PC에서는 웹 오퍼레이터의 가짜 로봇(`lekiwi-gui/web_operator/dev/fake_robot.py base`)이 드라이버의 팔 명령 · 토크 · `arm_ready` · `arm_registers`(lekiwi04 보정 값)를 흉내 낸다 (시작은 lekiwi04 접은 자세).
`ros2 topic pub --once fake/arm_torque_left_on std_msgs/msg/Bool '{data: true}'`이면 `torque_enabled`는 true인데 `arm_ready`는 false인 드라이버를 흉내 낸다 (이 노드는 `팔 토크가 꺼져 있습니다`로 멈춰 있어야 한다).
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`와 실물이 아닌 `ROS_DOMAIN_ID`로 띄우고, `lerobot_python:=`에 PC의 lerobot python(예: conda `lerobot060`)을 준다.

---

## 한계

- 보정 파일은 시작할 때 한 번 읽는다. 바꾸면 노드를 다시 켠다. 드라이버 레지스터(`arm_registers`)는 올 때마다 다시 맞춰 본다.
- 팔로워 자세는 30 Hz 모터 상태에서 온다. 램프 · 접은 자세 · 멈춘 뒤 첫 명령의 이동 제한은 그 자세를 기준으로 한다. 따라가는 중에는 지난 명령 기준 (따라가는 속도는 위).
- 이상 비트는 걸어 둔다: 비트가 내려가도 목표가 `goal_timeout_s` 넘게 끊길 때까지 (사람이 원격 조작을 멈출 때까지) 명령하지 않는다. 눌린 팔이 과부하 → 다시 출발을 되풀이하지 않게 한다.
  리더 목표가 무선 끊김으로 0.5초 넘게 끊겨도 풀린다. 팔 토크를 켠 채 비트가 내려가지 않는 이상(무게를 든 채 과열 · 전압)은 팔 토크를 꺼야만 풀린다.
- 이상이 걸리면 원격 조작 중이 아니어도 잰 자세를 한 번 보낸다: 무게를 버티던 관절은 그만큼 (P 오차) 처진다. ID가 다른 경우에는 보내지 않는다.
- 손목 돌림(`arm_wrist_roll`)의 ±180° 이음매를 다루지 않는다 (lerobot과 같다). 리더가 +179° → -179°로 넘어가면 팔로워는 반대쪽으로 358°를 돈다 (명령은 한 번에 45°씩, 팔은 서보 최고 속도로).
  손목을 이음매 너머로 돌리지 않는다.
- 그리퍼는 따라오는지 감시하지 않는다 (물건을 잡으면 막히는 것이 정상).
