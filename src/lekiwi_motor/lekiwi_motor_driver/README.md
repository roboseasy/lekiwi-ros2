# lekiwi_motor_driver

LeKiwi 바퀴 3개와 SO101 팔 6개가 함께 쓰는 **Feetech STS3215 버스를 여는 유일한 노드** 패키지.
모터 9개의 상태를 발행하고, 설정으로 켜면 **바퀴 3개**에 속도 · 토크를, **팔 6개**에 목표 위치 · 토크를 쓴다.
`lekiwi_motor` 계층의 첫 패키지다. 이름 규칙은 [이름 계약](../../docs/interfaces.md)을 따른다.

- 대상: Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · Feetech STS3215 × 9
- 워크스페이스: `~/lekiwi_ws/src/lekiwi_motor/lekiwi_motor_driver`
- 메시지: `lekiwi_msgs/MotorState`, `lekiwi_msgs/MotorStateArray`, `lekiwi_msgs/WheelCommand`, `lekiwi_msgs/ArmCommand`, `lekiwi_msgs/ArmRegisters`

> ✅ **기본은 읽기 전용**: `wheel_write_enabled: false` · `arm_write_enabled: false`(기본)면 READ 명령만 보낸다. 토크를 켜거나 모터를 움직이는 경로가 열리지 않는다.
>
> ⚠️ **바퀴 쓰기**(`wheel_write_enabled: true`)는 `lekiwi_bringup`이 베이스 컨트롤러와 함께 켤 때 쓴다.
> 바퀴 토크는 사람이 `lekiwi/motor/power`로 켜기 전까지 꺼져 있다. 바퀴 쓰기는 팔 모터(ID 1~6)에 아무것도 보내지 않는다.
>
> ⚠️ **팔 쓰기**(`arm_write_enabled: true`)는 `lekiwi_bringup`이 `use_arm:=true`(기본)일 때 켠다. 팔로워 팔(`lekiwi_lerobot`)이 쓴다.
> 시작할 때 팔 토크는 **건드리지 않는다**. 팔 명령은 `lekiwi/motor/arm_torque`로 토크를 켠 뒤에만 쓴다.
>
> ⚠️ 같은 버스를 쓰는 LeKiwi `lekiwi_node`(저장소 브링업)나 LeRobot 호스트(`start_lekiwi_host.sh`)와 **동시에 실행하지 않는다.**
> 이 노드는 실행 중 다른 프로그램이 포트를 새로 열지 못하게 막지만(`TIOCEXCL`), 이미 포트를 연 프로그램은 막지 못한다.

---

## 하드웨어 연결

모터 9개 → (데이지 체인) → USB 버스 서보 어댑터 → Raspberry Pi USB 포트.
**모터 전원(배터리)이 켜져 있어야** 모터가 응답한다.

```bash
ls -l /dev/serial/by-id/
# ... usb-1a86_USB_Single_Serial_...-if00 -> ../../ttyACM0
```

| ID | 이름 | 위치 |
|:--:|------|------|
| 1 | `arm_shoulder_pan` | 팔 |
| 2 | `arm_shoulder_lift` | 팔 |
| 3 | `arm_elbow_flex` | 팔 |
| 4 | `arm_wrist_flex` | 팔 |
| 5 | `arm_wrist_roll` | 팔 |
| 6 | `arm_gripper` | 팔 |
| 7 | `base_left_wheel` | 바퀴 |
| 8 | `base_back_wheel` | 바퀴 |
| 9 | `base_right_wheel` | 바퀴 |

> ID와 이름은 LeRobot LeKiwi 설정(`lekiwi.py`)과 같다. 바퀴 이름은 LeRobot 기준이며,
> 새 좌표계에서는 모델 바퀴 `left`, `back`, `right`가 ID `8`, `9`, `7`이다 (베이스 컨트롤러 `wheel_ids`).
> 실제 위치는 바퀴를 띄운 저속 시험으로 다시 확인한다.

---

## 노드

- `lekiwi_motor_driver_node`

## 구독 (Subscribe)

바퀴 토픽은 `wheel_write_enabled: true`, 팔 토픽은 `arm_write_enabled: true`일 때만 만든다. 둘 다 reliable, 깊이 1.

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/lekiwi/motor/wheel_command` | `lekiwi_msgs/WheelCommand` | 바퀴 ID별 속도 [rad/s]. 받으면 바로 SYNC_WRITE로 `Goal_Speed`(46)에 쓴다 |
| `/lekiwi/motor/arm_command` | `lekiwi_msgs/ArmCommand` | 팔 ID별 목표 위치 [tick, 0~4095]. 받으면 바로 SYNC_WRITE 한 번으로 메시지의 **모든 ID**를 `Goal_Position`(42)에 쓴다 (SYNC_WRITE는 ACK가 없어 걸러 내지 않는다) |

바퀴: 다음 명령은 쓰지 않고 버린다: 바퀴 토크가 꺼져 있을 때, `wheel_ids`에 없는 ID(팔 등)나 같은 ID가 두 번 있을 때, 속도가 유한하지 않을 때.
속도는 `speed_tick_scale`로 tick/s로 바꾸고 `speed_tick_limit`으로 크기를 자른다.

팔: 다음 명령은 쓰지 않고 버린다: 팔 토크를 `lekiwi/motor/arm_torque`로 켜지 않았을 때(다른 프로그램이 켜 둔 토크도 꺼진 것으로 본다), `ids`와 `positions_ticks` 길이가 다를 때,
`arm_ids`에 없는 ID(바퀴 등)나 같은 ID가 두 번 있을 때 (하나라도 있으면 메시지 전체), 이번 주기에 위치를 읽지 못한 팔(`online: false`)이 있을 때.
목표는 0~4095로 자른 뒤, 그 모터의 마지막 `Present_Position` ± `arm_max_goal_offset_ticks`(기본 1024 tick = 90°) 안으로 자른다 (경고 로그).
깨진 값 · 잘못된 발행자가 한 명령으로 팔을 반 바퀴 넘게 돌리지 못한다. 팔로워는 따라가는 중 리더 목표를 그대로 보내므로 (한 번에 지난 명령에서 45°까지),
빠른 리더를 서보가 뒤따라오는 차이보다 작게 두면 원격 조작이 느려진다 (2026-10-22 lekiwi04: 200이었을 때 팔이 200~305 ms 늦었다).
일부 ID만 보내도 된다 (받은 ID만 쓴다). 명령이 끊겨도 **아무것도 하지 않는다**: 모터는 마지막 목표를 유지한다.

## 발행 (Publish)

| 토픽 | 타입 | 주기 | 내용 |
|------|------|------|------|
| `/lekiwi/motor/states` | `lekiwi_msgs/MotorStateArray` | `publish_rate_hz` (기본 30 Hz) | 모터별 `MotorState`. 순서는 `motor_ids`와 같다. stamp는 그 주기에 모터를 다 읽은 시각 |
| `/lekiwi/motor/arm_ready` | `std_msgs/Bool` | 시작할 때 한 번 + 바뀔 때마다 | `true` = 팔 명령을 받는 중 (`arm_torque` true가 성공한 뒤). 시작 · `arm_torque` false · 켜기 실패 때 `false`. `arm_write_enabled: true`일 때만 |
| `/lekiwi/motor/arm_registers` | `lekiwi_msgs/ArmRegisters` | 시작할 때 한 번 + 다시 읽을 때 | 팔 EEPROM 값: `Homing_Offset`(31, 부호 비트 11을 푼 값 = LeRobot 보정 JSON의 `homing_offset`), `Min/Max_Position_Limit`(9, 11), `P/D/I_Coefficient`(21~23). 시작할 때 응답하지 않던 팔은 응답하면 읽어 다시 발행한다. **읽기만 한다** (EEPROM에 쓰지 않는다). `arm_write_enabled: true`일 때만 |

**`MotorState` 필드**

| 필드 | 단위 | 설명 |
|------|------|------|
| `id`, `name` | – | 모터 ID와 이름 |
| `online` | – | 이번 주기 읽기 성공. `false`면 `position` ~ `status`는 0 |
| `position_ticks` | tick | 0 ~ 4095 (한 바퀴 4096) |
| `position` | rad | `(position_ticks - 2048) × 2π / 4096`. **로봇 모델의 관절 각도가 아니다** |
| `velocity` | rad/s | 회전 속도 |
| `load` | % | 모터 구동 출력 비율. 부호는 방향. 토크 측정값이 아니다 |
| `voltage` | V | 모터 공급 전압 |
| `temperature` | °C | 모터 온도 |
| `current` | A | 모터 전류 |
| `moving` | – | 움직이는 중 |
| `status` | – | 보호 상태 비트. 0이면 정상 (`STATUS_VOLTAGE`, `STATUS_OVERHEAT`, `STATUS_OVERLOAD` 등) |
| `mode` | – | 운전 모드 (`MODE_POSITION`=0, `MODE_VELOCITY`=1, …). 1초마다 갱신 |
| `torque_enabled` | – | 토크 켜짐. 1초마다 갱신하고, 이 노드가 토크를 쓰면 바로 바뀐다 |

- QoS: `states`는 `SensorDataQoS` (best effort). `arm_ready` · `arm_registers`는 reliable + transient_local, 깊이 1 (`lekiwi/motor/ready`와 같다. 나중에 붙어도 마지막 값을 받는다)

## 서비스

`wheel_torque`는 `wheel_write_enabled: true`일 때만 만든다. **호출하는 쪽은 베이스 컨트롤러뿐이다.** 사람은 `lekiwi/motor/power`를 쓴다.
`arm_torque`는 `arm_write_enabled: true`일 때만 만든다. 게이트웨이(시작 · 팔 힘 빼기 · 비상 정지)가 부른다 (팔로워 팔은 부르지 않는다).

| 서비스 | 타입 | 동작 |
|--------|------|------|
| `/lekiwi/motor/wheel_torque` | `std_srvs/SetBool` | `true`: 바퀴마다 지금 다시 읽어 응답 · 속도 모드(1) · 보호 비트 0을 확인 → 속도 0 → 토크 켬(ACK) → 다시 읽어 켜졌는지 확인. 하나라도 실패하면 다시 끄고 `success: false`<br>`false`: 속도 0 → 토크 끔(ACK). 확인하지 못한 바퀴는 1초마다 다시 끈다 |
| `/lekiwi/motor/arm_torque` | `std_srvs/SetBool` | `true`: 이미 `arm_ready`이고 팔을 다시 읽어 모두 토크가 켜져 있고 보호 비트가 없으면 **아무것도 쓰지 않고** 성공 (두 번 불러도 안전, `arm_ready`도 다시 내지 않는다).<br>아니면 `arm_ready=false`로 두고, 끄기를 확인하지 못한 팔이 있으면 거부. 팔마다 지금 다시 읽어 응답 · 위치 모드(0) · 보호 비트 0 · 위치 0~4095를 확인(하나라도 아니면 아무것도 쓰지 않고 거부) → 그때의 토크 상태를 기억 → **꺼져 있던 팔만** `Goal_Position` = 지금 위치(WRITE, ACK) → 목표를 쓴 팔만 토크 켬(ACK). 잡고 있던 팔은 목표를 다시 쓰지 않는다 (무게에 눌린 만큼 처지지 않게). 실패한 ID는 한 번 더. 그래도 실패하면 **호출 전에 꺼져 있던 팔만** 다시 끄고(잡고 있던 팔은 끄지 않는다) `success: false`와 ID. 성공하면 `arm_ready=true`. 켜는 순간 남아 있던 옛 목표로 튀지 않는다<br>`false`: `arm_ready=false` → 팔 ID에만 하나씩 토크 끔(ACK). 브로드캐스트를 쓰지 않고 바퀴는 건드리지 않는다. 응답이 없는 ID는 `success: false`로 알리고 1초마다 그 ID에만 다시 끈다 |

바퀴 · 팔 모드(EEPROM)는 쓰지 않는다. 바퀴가 속도 모드가 아니거나 팔이 위치 모드가 아니면 토크를 켜지 않고 이유를 돌려준다.

## 안전 감시 (토크가 켜진 동안 매 주기)

| 상황 | 동작 |
|------|------|
| 노드 시작 (`wheel_write_enabled: true`) | 다른 프로그램이 남긴 바퀴 토크 · 속도를 정리: 속도 0 → 토크 끔. 팔은 건드리지 않는다 |
| 바퀴 명령이 `wheel_command_timeout`(0.5 s) 넘게 없음 | 속도 0을 계속 쓴다 (로그 한 번) |
| 바퀴 명령이 `wheel_torque_off_timeout`(3 s) 넘게 없음 | 속도 0 → 토크 끔 |
| 바퀴 하나가 `offline_after_failures`번(5번, 30 Hz 약 167 ms) 연속 응답하지 않음 | 모든 바퀴 속도 0 → 토크 끔 |
| 바퀴 상태 비트(전압 · 각도 센서 · 과열 · 과전류 · 과부하)가 켜짐 | 모든 바퀴 속도 0 → 토크 끔 |
| 바퀴 토크가 이 노드 밖에서 꺼짐 | 모든 바퀴 속도 0 → 토크 끔 |
| 노드 종료 (`Ctrl+C`, 예외) | 바퀴 토크가 켜져 있었으면 속도 0 → 토크 끔. **팔 토크는 그대로 둔다** (팔이 떨어지지 않게 마지막 목표 유지) |

팔에는 감시가 없다: 팔 명령이 끊겨도, 팔이 응답하지 않아도 드라이버는 팔 토크를 바꾸지 않는다. 팔 토크를 끄는 것은 `arm_torque` false
(와 켜기 실패 때 원래 꺼져 있던 팔 되돌리기)뿐이다. 확인하지 못한 팔 끄기는 1초마다 그 ID에만 다시 보낸다.

## 파라미터

설정 파일: `config/lekiwi_motor_driver.yaml`

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `port` | string | `/dev/ttyACM0` | 버스 시리얼 포트. launch 인자 `port`가 덮어쓴다 |
| `baud_rate` | int | `1000000` | 통신 속도 |
| `read_timeout_ms` | int | `10` | 모터 한 개의 응답을 기다리는 최대 시간 [ms] |
| `motor_ids` | int[] | `[1, …, 9]` | 읽을 모터 ID |
| `motor_names` | string[] | 위 표 | `motor_ids`와 같은 순서의 이름 |
| `publish_rate_hz` | double | `30.0` | 위치 · 속도 · 부하 · 전압 · 온도 · 전류 · 상태 읽기 주기. lerobot LeKiwi 데이터셋(30 fps)과 같다. 주기 예산은 아래 |
| `slow_read_period` | double | `1.0` | [s] 모터마다 운전 모드 · 토크 상태 갱신, 응답 없는 모터 재시도 주기. `slow_read_period` / 모터 수마다 한 모터씩 차례로 한다 (주기마다 많아야 하나). 확인 못 한 토크 끄기 다시 보내기 · 팔 EEPROM 다시 읽기도 이 주기 |
| `offline_after_failures` | int | `5` | 연속 실패가 이만큼 쌓이면 응답 없음으로 본다 (30 Hz 약 167 ms, 20 Hz 3번 150 ms와 같은 시간). 그 뒤 0.3 s(`kOfflineRetryGrace`)는 주기마다, 그다음은 느린 읽기 차례(1초에 한 번)에만 다시 읽는다. 다시 응답하면 운전 모드 · 토크도 바로 읽는다 |
| `states_topic` | string | `lekiwi/motor/states` | 발행 토픽 |
| `wheel_write_enabled` | bool | `false` | 바퀴 쓰기 토픽 · 서비스를 만든다. launch 인자 `wheel_write_enabled`가 덮어쓴다 |
| `wheel_ids` | int[] | `[7, 8, 9]` | 쓰기를 허용하는 모터. `motor_ids` 안에 있어야 한다 |
| `speed_tick_scale` | double | `651.8986…` | [tick/s per rad/s] = 4096 / 2π |
| `speed_tick_limit` | int | `3000` | [tick/s] `Goal_Speed` 크기 상한 (약 4.60 rad/s). STS3215 · LeRobot 상한(max_raw 3000). 2026-10-22 (사용자 요청 0.3 m/s · 90°/s → 베이스 상한 0.23 m/s · 1.53 rad/s(88°/s)). 코드 기본값은 1500 |
| `wheel_command_timeout` | double | `0.5` | [s] 명령이 끊기면 속도 0 |
| `wheel_torque_off_timeout` | double | `3.0` | [s] 명령이 끊기면 토크 끔. `wheel_command_timeout` 이상 |
| `wheel_command_topic` | string | `lekiwi/motor/wheel_command` | 바퀴 명령 토픽 |
| `wheel_torque_service` | string | `lekiwi/motor/wheel_torque` | 바퀴 토크 서비스 |
| `arm_write_enabled` | bool | `false` | 팔 쓰기 토픽 · 서비스를 만든다. `lekiwi_bringup` `use_arm:=true`가 덮어쓴다 |
| `arm_ids` | int[] | `[1, 2, 3, 4, 5, 6]` | 쓰기를 허용하는 팔 모터. `motor_ids` 안에 있고 `wheel_ids`와 겹치지 않아야 한다 |
| `arm_max_goal_offset_ticks` | int | `1024` | [tick] 팔 목표를 그 모터의 마지막 `Present_Position` ± 이 값 안으로 자른다 (90°). 1 ~ 4095. 빠른 원격 조작의 서보 뒤처짐보다 커야 한다 |
| `arm_command_topic` | string | `lekiwi/motor/arm_command` | 팔 명령 토픽 |
| `arm_torque_service` | string | `lekiwi/motor/arm_torque` | 팔 토크 서비스 |
| `arm_ready_topic` | string | `lekiwi/motor/arm_ready` | 팔 명령을 받는 중인지 (`std_msgs/Bool`) |
| `arm_registers_topic` | string | `lekiwi/motor/arm_registers` | 팔 EEPROM 값 (`lekiwi_msgs/ArmRegisters`) |

`arm_*` 값은 `arm_write_enabled: true`일 때만 읽고 검증한다 (팔 쓰기를 끈 설정의 `arm_ids`가 읽기 전용 시작을 막지 않는다).

---

## 빌드

메시지 패키지 `lekiwi_msgs`를 함께 빌드한다.

```bash
cd ~/lekiwi_ws
cbs lekiwi_msgs lekiwi_motor_driver && sb
```

> ⚠️ lerobot venv가 켜진 셸에서 `colcon build`를 직접 실행하면 `No module named 'catkin_pkg'`로 실패한다.
> 먼저 `deactivate` 하거나 `cb` / `cbs` / `cbsource`를 사용한다.

`config/` · `launch/` 파일은 `--symlink-install`로 연결되므로 수정 후 노드만 재시작하면 적용된다. `.cpp` · `.hpp`를 고쳤다면 다시 빌드한다.

## 시험 (하드웨어 없음)

```bash
cd ~/lekiwi_ws
colcon test --packages-select lekiwi_motor_driver --event-handlers console_direct+
colcon test-result --verbose
```

- `test_feetech_protocol`: READ · WRITE · SYNC_WRITE 패킷, 체크섬, 부호-크기 변환, 속도 → tick, 목표 위치 → 2바이트 (0~4095로 자름)
- `test_wheel_write_pty`: 가상 Feetech 버스(PTY)에 노드를 붙여 읽기 전용 기본값, 시작 시 남은 바퀴 토크 정리, 토크 꺼짐 시 명령 무시, 속도 모드 · 상태 비트 확인, 0.5 s · 3 s 감시, 팔 ID 거부, 바퀴 응답 끊김 · 외부 토크 꺼짐 시 전체 정지, 종료 시 토크 끔을 확인한다.
  팔: 시작 · 종료 때 팔 토크를 건드리지 않음, 토크를 켜기 전 명령 무시, 토크 켜기는 목표 = 지금 위치를 먼저 쓰고 팔 ID에만 WRITE, 위치 모드가 아니면 거부, 응답 없음 · 보호 비트면 거부(쓰기 없음), 켜진 상태에서 다시 켜기는 아무것도 쓰지 않음, 실패한 ID만 한 번 더, 두 번 실패하면 원래 꺼져 있던 팔만 되돌림, 토크 끄기 실패는 1초마다 그 ID에만 다시, 명령마다 모든 목표를 한 번의 SYNC_WRITE로 쓰기 · 범위 밖 · 지금 위치 ± 1024 밖 자르기, 명령이 끊긴 뒤 3 s 넘게 아무것도 쓰지 않음, 바퀴 · 같은 ID가 섞인 명령 버림, 토크 끄기는 팔 ID에만, `arm_ready` 변화(시작 false → 켜기 true → 끄기 · 켜기 실패 false), `arm_registers` 값(음수 Homing_Offset 포함, 늦게 응답한 팔 다시 읽기), 팔 쓰기를 켰을 때만 `arm_ids` · `wheel_ids` 겹침 거부, 바퀴 · 팔 명령을 섞어도 각자 자기 모터에만 쓰는지
  주기 예산: 응답마다 2 ms 늦게 답하는 가상 버스(따로 프로세스)에서 설정 파일의 `publish_rate_hz`(30)로 바퀴 50 Hz · 팔 30 Hz 명령을 보내며 5.5 s 동안. 판정은 주기마다 상태 9개 · 느린 읽기 많아야 1개, 모터마다 운전 모드 · 토크를 1초마다 읽음. 시간은 CPU 부하에 흔들려 느슨하게만 본다: states 30 Hz ± 20 %, 상태 95 %가 모두 online, 버스 쪽 주기 바쁜 시간 가운데값 < 50 ms, 보낸 명령의 30 % 넘게 SYNC_WRITE로 버스에 감 (모든 코어를 바쁘게 돌려도 5번 다 통과)
  짧은 끊김: 팔 모터 하나가 130 ms 응답하지 않아도 응답 없음으로 보지 않고, 250 ms면 응답 없음이 되지만 (0.3 s 동안 주기마다 다시 읽어) 둘 다 끊김이 끝난 뒤 0.2 s 안에 돌아온다
- 가상 버스 `test/fake_sts_bus.py`: `FakeStsBus(reply_delay=…)`로 응답을 늦출 수 있고, `python3 fake_sts_bus.py <지연 s>`로 따로 프로세스로 띄우면 포트를 쓰고 표준 입력이 닫힐 때 READ · 쓰기 기록을 JSON으로 쓴다
- 시험이 쓰는 ROS 도메인은 기본으로 프로세스 번호에서 고른다 (80~99). `LEKIWI_TEST_ROS_DOMAIN_ID`로 정할 수 있다

---

## 실행

읽기 전용으로 상태만 볼 때:

```bash
ros2 launch lekiwi_motor_driver lekiwi_motor_driver.launch.py
```

바퀴를 움직이려면 이 launch가 아니라 `lekiwi_bringup`(베이스 컨트롤러 포함)을 쓴다.

모터가 모두 응답하면 이런 로그가 나온다.

```text
Feetech bus open on /dev/ttyACM0 at 1000000 baud (read only: no torque or motion commands)
motor 1 (arm_shoulder_pan): model 777 (STS3215)
...
motor 9 (base_right_wheel): model 777 (STS3215)
9/9 motors responded
publishing /lekiwi/motor/states at 30.0 Hz
```

응답하지 않는 모터는 `motor N (...): no response`로 표시되고, 1초마다 다시 찾는다 (실행 중에 끊긴 모터는 처음 0.3 s 동안 주기마다). 종료는 `Ctrl+C`.

`lekiwi_bringup`으로 켜면 첫 줄이 `(wheel and arm write enabled)`로 바뀌고, 쓰기가 준비되면 이런 줄이 더 나온다.

```text
wheel write ready for ids [7, 8, 9]: /lekiwi/motor/wheel_command (rad/s), /lekiwi/motor/wheel_torque (SetBool); torque stays off until requested
arm write ready for ids [1, 2, 3, 4, 5, 6]: /lekiwi/motor/arm_command (Goal_Position ticks), /lekiwi/motor/arm_torque (SetBool); arm torque is left as it is until requested
```

**launch 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | `share/lekiwi_motor_driver/config/lekiwi_motor_driver.yaml` | 파라미터 파일 |
| `port` | `/dev/ttyACM0` | 버스 시리얼 포트 |
| `wheel_write_enabled` | `false` | 바퀴 쓰기를 연다 |

---

## 확인 명령어

노드를 켠 상태에서 **다른 터미널**에서 실행한다.

| 명령어 | 확인 내용 |
|--------|----------|
| `ros2 topic list` (`rt`) | `/lekiwi/motor/states`가 보이는지 |
| `ros2 topic hz /lekiwi/motor/states` | 약 30 Hz인지 |
| `ros2 topic echo --once /lekiwi/motor/states` | 9개 모터 값을 한 번 보기 |
| `ros2 interface show lekiwi_msgs/msg/MotorState` | 필드 · 상수 설명 보기 |
| `ros2 param get /lekiwi_motor_driver_node wheel_write_enabled` | 바퀴 쓰기가 열렸는지 |
| `ros2 param get /lekiwi_motor_driver_node arm_write_enabled` | 팔 쓰기가 열렸는지 |
| `ros2 service list \| grep arm_torque` | `/lekiwi/motor/arm_torque`가 보이는지 (팔 쓰기를 열었을 때) |

**정지 상태의 정상 값**: 모든 모터 `online: true`, `status: 0`, `voltage`는 배터리 전압과 비슷, `temperature`는 실온 근처.

**손으로 확인하기**: `torque_enabled: false`인 팔 관절을 손으로 천천히 움직이면 그 모터의 `position_ticks`가 바뀐다.
바퀴를 손으로 돌리면 `velocity`가 바뀐다. `torque_enabled: true`인 모터는 손으로 억지로 돌리지 않는다.

---

## 처리 흐름

```text
Feetech 버스 (/dev/ttyACM0, 1 Mbps, 모터 9개)
  ├─ 시작: 모터마다 모델 번호 읽기 → 응답 여부 · STS3215 확인 (로그)
  ├─ 30 Hz: 응답하는 모터마다 레지스터 56~70 (15바이트) 읽기
  │          → 위치 · 속도 · 부하 · 전압 · 온도 · 상태 · 이동 여부 · 전류
  │          → (바퀴 쓰기를 켰으면) 안전 감시
  ├─ 모터마다 1초에 한 번 (1/9초마다 한 모터씩): 레지스터 33~40 읽기 → 운전 모드 · 토크 상태,
  │        응답 없는 모터면 다시 찾기 (막 끊긴 모터는 0.3 s 동안 주기마다)
  ├─ 1 Hz: 확인 못 한 토크 끄기 다시 보내기, 늦게 응답한 팔 EEPROM 읽기
  └─ MotorStateArray ──────────────────────────→ /lekiwi/motor/states

/lekiwi/motor/wheel_command ──→ (토크 켜짐 · 바퀴 ID 확인) ──→ SYNC_WRITE Goal_Speed (바퀴 3개)
/lekiwi/motor/wheel_torque  ──→ 확인 → 속도 0 → WRITE Torque_Enable (바퀴 3개, ACK)
/lekiwi/motor/arm_command   ──→ (arm_ready · 팔 ID 확인 · 0~4095, 지금 위치 ± 1024 자름) ──→ SYNC_WRITE Goal_Position (받은 팔 모두)
/lekiwi/motor/arm_torque    ──→ 확인 → WRITE Goal_Position = 지금 위치 → WRITE Torque_Enable (팔 6개, ACK) ──→ /lekiwi/motor/arm_ready
시작 (arm 쓰기) ──→ 팔마다 레지스터 9~32 읽기 (EEPROM, 읽기만) ──→ /lekiwi/motor/arm_registers
```

모든 콜백(읽기 타이머, 바퀴 · 팔 명령, 토크 서비스)이 기본 콜백 그룹 하나에서 한 스레드(`rclcpp::spin`)로 차례로 돌아
버스 접근이 겹치지 않는다. 잠금이 따로 없으므로 콜백 그룹이나 MultiThreadedExecutor를 더하지 않는다.

### 주기 예산 (30 Hz = 33.3 ms)

한 주기의 버스 일: 상태 READ 9번 + 느린 읽기 READ 많아야 1번 = 응답을 기다리는 거래 **10번**.
바퀴 SYNC_WRITE(50 Hz) · 팔 SYNC_WRITE(약 30 Hz)는 응답이 없어 기다리지 않는다 (같은 스레드에서 주기 사이에 쓴다).
그래서 거래 한 번(요청 → 응답)이 약 3.3 ms보다 짧으면 주기 안에 든다.
예전처럼 1초에 한 번 모터 9개의 운전 모드 · 토크를 한 주기에 몰아 읽으면 그 주기는 거래 18번이 되어, 거래가 1.85 ms만 넘어도 주기를 넘긴다
(그 주기는 경고도 내지 않았다). 그래서 느린 읽기를 모터 수로 나눴다.

가상 버스(PTY, 응답마다 지연, 바퀴 50 Hz · 팔 30 Hz 명령을 보내며 5.5 s) 측정 (2026-10-22, PC). 바쁜 시간은 모터 1 상태 요청 ~ 마지막 응답:

| 응답 지연 | 주기 바쁜 시간 평균 · 최대 | 결과 |
|------|------|------|
| 1 ms | 12.5 · 15.0 ms | 통과 |
| 2 ms (시험 기본) | 22.4 · 25.6 ms | 통과. 나누기 전 코드는 느린 주기가 46.1 ms (경고 없음) |
| 3 ms | 32.1 · 35.6 ms | 넘침 (`reading motors took ...` 경고) |

실물 거래 시간은 아직 재지 않았다. USB 왕복(1 ms 프레임) + 1 Mbps 전송(READ 한 번 요청 8 + 응답 21바이트, 약 0.3 ms)이면 1~2 ms로 어림한다.
Pi에서 `reading motors took ...` 경고가 나오면 `publish_rate_hz`를 낮추기 전에 거래 시간을 잰다 (sync read로 바꾸면 READ 9번의 USB 왕복이 한 번으로 준다).
읽기가 실패한 주기(`read_timeout_ms` 10 ms를 기다렸다)와 느린 일(확인 못 한 토크 끄기 다시 보내기 · 읽지 못한 팔 EEPROM 읽기)을 한 주기는 경고하지 않는다.
응답 없는 모터 하나는 주기마다 10 ms를 더한다: 막 끊긴 뒤 약 0.47 s(5번 실패 + 0.3 s) 동안은 주기마다, 그 뒤로는 1초에 한 번.
거래 2 ms면 8 × 2 + 10 = 26 ms로 주기 안이다. 팔 6개가 한꺼번에 끊기면 그 0.47 s 동안 주기가 약 70 ms로 늘어난다 (states 약 15 Hz, 베이스 컨트롤러의 0.5 s 감시 안).

## 알려진 한계

- **바퀴 쓰기는 가상 버스로만 시험했다.** 실물 바퀴 토크 · 속도는 바퀴를 띄운 저속 시험으로 확인해야 한다.
- `position`은 센서 원점(2048) 기준 각도다. 팔 관절 각도 · LeRobot 보정 각도와 다르다. 팔 `joint_states`(RViz 연동)는 보정값 대응을 확인한 뒤 추가한다.
- `current` 단위(1 = 6.5 mA)는 Feetech 문서 기준이며 실측으로 확인하지 않았다.
- 모터를 한 개씩 차례로 읽는다(sync read 미사용). 30 Hz는 거래 한 번이 약 3.3 ms보다 짧아야 한다 (위 '주기 예산'). 주기가 모자라면 `reading motors took ...` 경고가 나온다.
- **팔 쓰기는 가상 버스로만 시험했다.** 팔 목표 SYNC_WRITE는 응답이 없어 도착을 확인할 수 없다. 그래서 명령마다 모든 목표를 다시 쓰고, 깨진 패킷은 다음 명령(약 30 Hz)이 덮어쓴다. 팔 P 계수 · 가속도 등은 쓰지 않고 모터에 남은 값(LeRobot이 설정한 값)을 그대로 쓴다.
- 팔은 토크가 켜진 동안 감시하지 않는다: 팔 하나가 응답하지 않거나 밖에서 토크가 꺼져도 `arm_ready`는 true로 남는다 (그 팔의 위치를 읽지 못하면 명령은 버린다).
- USB를 뽑았다 다시 꽂으면 노드를 재시작해야 한다.

---

## 문제 해결

| 증상 | 원인 · 조치 |
|------|------------|
| `Package 'lekiwi_motor_driver' not found` | 워크스페이스를 source하지 않음. `sb` |
| 빌드 중 `lekiwi_msgs`를 찾지 못함 | 메시지 패키지를 함께 빌드: `cbs lekiwi_msgs lekiwi_motor_driver` |
| `failed to open /dev/ttyACM0: No such file or directory` | 어댑터 USB 연결 확인. `ls -l /dev/serial/by-id/` |
| `failed to open ...: Permission denied` | 사용자에게 `dialout` 그룹이 필요. `id`로 확인 |
| `failed to open ...: Device or resource busy` | 다른 프로그램이 포트를 독점 중. 이 노드를 두 번 실행하지 않았는지 확인 |
| `no motor responded` | 모터 전원(배터리) 꺼짐, 포트 · `baud_rate` 확인. 전원을 켜면 1초 안에 다시 찾는다 |
| 일부 모터만 `no response` | 그 모터의 케이블 · ID 확인 |
| `stopped responding after ... failed reads` | 케이블 접촉 불량 또는 전원 문제 |
| `status 0x..: overheat` 등 | 모터 보호 상태. 로봇을 멈추고 원인을 확인한다 |
| `refused: wheel N ... not velocity mode (1)` | 바퀴가 속도 모드가 아니다. LeRobot 설정 도구로 바퀴를 속도 모드로 맞춘다 (이 노드는 모드를 쓰지 않는다) |
| `wheel command ignored: wheel torque is off` | 토크를 켜지 않았다. 베이스 컨트롤러의 `lekiwi/motor/power`로 켠다 |
| `no wheel command for ...` | 베이스 컨트롤러가 멈췄거나 명령이 끊겼다. 드라이버가 속도 0 · 토크 끔으로 정리한다 |
| `reading motors took ... ms` | 주기가 모자람 (거래 한 번이 약 3.3 ms 넘음, '주기 예산'). 같은 버스를 다른 프로그램이 쓰는지 · 케이블을 확인하고, 그래도면 `publish_rate_hz`를 낮춘다 (lerobot 데이터셋은 30 fps다) |
| `arm command ignored: arm is not ready ...` | 팔 토크를 켜지 않았거나 켜기가 실패했다 (`/lekiwi/motor/arm_ready`가 false). `lekiwi/motor/arm_torque` true를 부른다 (게이트웨이가 부른다) |
| `refused: arm torque disable of ids [...] is not confirmed yet` | 앞선 팔 끄기에 응답하지 않은 팔이 있다. 그 ID에 1초마다 다시 끄고, 확인되면 다시 켤 수 있다 |
| `goals more than 1024 ticks from the present position were clamped` | 팔 명령이 지금 위치에서 너무 멀다. 보내는 쪽(팔로워 램프 · 이동 제한)을 확인한다 |
| 값이 가끔 끊기거나 튐 | 같은 버스를 다른 프로그램이 동시에 쓰는지 확인: `fuser -v /dev/ttyACM0` |

---

## 패키지 구조

```text
lekiwi_motor_driver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lekiwi_motor_driver.yaml       # 파라미터
├── launch/lekiwi_motor_driver.launch.py  # 실행 파일
├── include/lekiwi_motor_driver/
│   ├── feetech_bus.hpp                   # Feetech 시리얼 버스 · 패킷
│   ├── sts3215.hpp                       # 레지스터 주소 · 값 변환
│   └── lekiwi_motor_driver_node.hpp      # ROS 2 노드
├── src/
│   ├── feetech_bus.cpp
│   └── lekiwi_motor_driver_node.cpp      # main 포함
└── test/
    ├── test_feetech_protocol.cpp         # 패킷 · 변환 gtest
    └── test_wheel_write_pty.py           # 가상 버스 시험
```

## 참고한 코드

- **LeRobot `motors/feetech/tables.py`**: STS3215 레지스터 주소, 부호 규칙, 모델 번호 777
- **LeRobot `robots/lekiwi/lekiwi.py`**: 모터 ID · 이름, `/dev/ttyACM0`, 1 Mbps
- **scservo_sdk**: Status 비트 (`ERRBIT_*`)
- **LeKiwi 저장소 `lekiwi_node/src/feetech_motor_interface.cpp`**: 시리얼 설정, 패킷 · 체크섬, 속도 단위 (tick/s, 한 바퀴 4096), 바퀴 토크 켜기 순서(속도 0 → 토크 켬, ACK 확인)
- **lekiwi_imu**: 패키지 구조, 파라미터 선언 · 검증, 로그 형식

## 검증 기록

- 2026-10-17, lekiwi04 (Pi 4, Jazzy): `lekiwi_msgs`와 함께 빌드 성공, 빌드 경고 0. 없는 포트로 실행하면 `failed to open`으로 종료됨
- 2026-10-17, lekiwi04 실물 읽기 (모터 전원 켜짐, 로봇 정지, 토크 · 이동 명령 없음). `/lekiwi/motor/states` 한 메시지에서 확인:

  | ID | 이름 | online | 전압 | 온도 | status | mode | torque_enabled |
  |:--:|------|:------:|:----:|:----:|:------:|:----:|:--------------:|
  | 6 | `arm_gripper` | – | – | – | 0 | 0 (위치) | false |
  | 7 | `base_left_wheel` | true | 12.1 V | 36 °C | 0 | 1 (속도) | false |
  | 8 | `base_back_wheel` | true | 12.0 V | 37 °C | 0 | 1 (속도) | false |
  | 9 | `base_right_wheel` | true | 11.9 V | 35 °C | 0 | 1 (속도) | false |

- 2026-10-18, PC (Jazzy): 바퀴 쓰기 추가 후 빌드 경고 0. `test_feetech_protocol` 8개, `test_wheel_write_pty` 10개 통과 (가상 버스, 하드웨어 없음)
- 2026-10-22, PC (Jazzy): 팔 쓰기 추가 후 빌드 경고 0. `test_feetech_protocol` 10개, `test_wheel_write_pty` 17개(팔 7개 포함) 통과 (가상 버스, 하드웨어 없음)
- 2026-10-22, PC (Jazzy): `arm_ready` · `arm_registers` · 다시 켜기 멱등 · 실패 되돌리기 · 끄기 재시도 · 목표 제한 추가 후 빌드 경고 0. `test_feetech_protocol` 10개, `test_wheel_write_pty` 27개(팔 17개 포함) 통과. 멱등 · 되돌리기 범위 · 필터 없음 · 목표 제한 · 끄기 재시도를 일부러 깨면 해당 시험이 실패함을 확인 (가상 버스, 하드웨어 없음)
- 2026-10-22, PC (Jazzy): 응답 없음 판단 5번 · 막 끊긴 모터 0.3 s 다시 읽기 · 주기 시각 · 경고 조건 정리 후 빌드 경고 0. `test_wheel_write_pty` 31개 통과. 짧은 끊김 시험은 판단 3번이면 (130 ms에 응답 없음), 다시 읽기가 없으면 (250 ms 끊김 뒤 415~447 ms에야 돌아옴) 실패함을 확인
- 2026-10-22, PC (Jazzy): 발행 주기 30 Hz · 느린 읽기 나누기 후 빌드 경고 0. `test_feetech_protocol` 10개, `test_wheel_write_pty` 29개(주기 예산 1개 포함) 통과. 주기 예산 시험은 나누기 전 코드에서 실패함을 확인 (느린 주기 46.1 ms, 응답 지연 2 ms, 가상 버스)
- 팔 ID 1~5의 값, 발행 주기(30 Hz), 실물 버스 거래 시간, 손으로 움직였을 때의 값 변화, 전류 단위, **실물 바퀴 토크 · 속도 쓰기**, **실물 팔 토크 · 위치 쓰기**는 아직 기록하지 않았다.
