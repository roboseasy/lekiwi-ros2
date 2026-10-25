# lekiwi_teleop

LeKiwi 베이스를 **PC에서 조작하고 시험하는 도구** 모음. 키보드 조작 창, 짧은 속도 명령, Nav2 모터 감독, 거리 시험, odom 보정 계산이 들어 있다.

- 실행 위치: **PC** (Ubuntu 24.04 · ROS 2 Jazzy). Pi에는 설치하지 않아도 된다
- 상대 노드: Pi의 `lekiwi_base_controller_node`(베이스 컨트롤러), `lekiwi_motor_driver_node`(모터 드라이버)
- 이름 기준: [`docs/interfaces.md`](../docs/interfaces.md). 코드 안의 이름은 `lekiwi_teleop/interface_names.py` 한 곳에 모았다
- 메시지: 표준 메시지 · 서비스만 쓴다. `lekiwi_msgs`에 의존하지 않는다
- 출처: LeKiwi 저장소 `feature/lekiwi-ros2`의 `lekiwi_teleop`. 노드 · 토픽 · 서비스 이름만 이 워크스페이스에 맞게 바꿨다

> ⚠️ **실물 바퀴 토크를 켜는 도구가 있다.** `--yes-i-confirm-motion`은 "현장 확인을 마쳤다"는 표시일 뿐 승인이 아니다.
> 실행 전에 로봇 주변, 즉시 전원을 끊을 사람, 바퀴를 띄울지 바닥에서 움직일지를 현장에서 확인한다.
>
> ⚠️ 서비스 성공 응답이나 종료 메시지는 물리 정지를 뜻하지 않는다. 바퀴가 멈췄는지 눈으로 확인한다.

---

## 실행 파일

| 실행 파일 | 실행 위치 | 필요한 것 | 바퀴 토크 | 용도 |
|---|---|---|---|---|
| `lekiwi_guarded_base_teleop` | PC (그래픽 데스크톱) | `python3-tk`, 베이스 컨트롤러 | 켜고, 끝나면 끈다 | 실물 키보드 주행 |
| `lekiwi_keyboard_teleop` | 창 (Pi에서 `ssh -X`, 또는 PC) | `python3-tk`, 베이스 컨트롤러 | 켜고, 끝나면 끈다 | **실물 키보드 주행 (권장).** 동시 입력 (`w`+`d` 대각선) |
| `lekiwi_terminal_teleop` | 터미널 (Pi SSH 포함) | 베이스 컨트롤러 | 켜고, 끝나면 끈다 | 실물 키보드 주행 (그래픽 창 없이) |
| `lekiwi_safe_cmd_vel` | PC 터미널 | 베이스 컨트롤러 | `--arm-motors`일 때만 | 정해진 시간만 속도 명령 |
| `lekiwi_nav2_motor_guard` | Pi (Nav2 launch가 실행) | Nav2 `collision_monitor`, 스캔 필터, 베이스 컨트롤러 | 켜고, 끝나면 끈다 | Nav2 주행 중에만 토크 유지 |
| `lekiwi_distance_trial` | PC 터미널 | 실물 `run`: 베이스 컨트롤러 + 모터 드라이버. `--mock`: mock 베이스만 | 실물 `run`일 때만 | 짧은 직선 이동의 odom · 실측 비교 |
| `lekiwi_odom_calibration` | PC 터미널 (ROS 연결 없음) | 없음 | 건드리지 않음 | `encoder_odom_scale` 후보 계산 |
| `lekiwi_base_teleop` | PC 터미널 | mock 베이스 | 건드리지 않음 | mock 전용 데모 |

## 사용하는 인터페이스

| 이름 | 타입 | 방향 | 쓰는 실행 파일 |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | 발행 | 조작 창, `safe_cmd_vel`, `distance_trial`, `base_teleop`. Nav2 감독은 토크를 켜기 전과 종료할 때 zero만 보낸다 |
| `lekiwi/motor/power` | `std_srvs/SetBool` | 호출 | 조작 창, `safe_cmd_vel --arm-motors`, Nav2 감독, `distance_trial run` (실물) |
| `lekiwi/motor/ready` | `std_msgs/Bool` (reliable + transient_local, 깊이 1) | 구독 | 조작 창, `safe_cmd_vel --arm-motors`, Nav2 감독, `distance_trial run` |
| `odom` | `nav_msgs/Odometry` | 구독 | Nav2 감독, `distance_trial run` |
| `lekiwi/sensor/scan_filtered` | `sensor_msgs/LaserScan` (SensorData) | 구독 | Nav2 감독 |
| `/lekiwi_base_controller_node` 파라미터 | 파라미터 읽기 | – | Nav2 감독, `distance_trial run` |
| `/lekiwi_motor_driver_node` 파라미터 `speed_tick_scale`, `speed_tick_limit` | 파라미터 읽기 | – | `distance_trial run` |

- 모두 상대 이름이다. namespace 없이 실행하면 `/cmd_vel`, `/lekiwi/motor/power`처럼 풀린다.
  노드 그래프 검사(`/collision_monitor`, `/lekiwi_base_controller_node`)와 파라미터 읽기도 도구 노드와 같은 namespace를 기준으로 한다.
- 사람이 바퀴 토크를 켤 때는 모터 드라이버의 `lekiwi/motor/wheel_torque`가 아니라 베이스 컨트롤러의 `lekiwi/motor/power`를 쓴다. 이 패키지의 도구도 `lekiwi/motor/power`만 부른다.

---

## 설치 · 빌드 (PC)

```bash
sudo apt install python3-tk             # 조작 창. rosdep install로도 설치된다

# PC 워크스페이스 루트에서
cbs lekiwi_teleop && sb
# alias가 없으면
colcon build --symlink-install --packages-select lekiwi_teleop && source install/setup.bash
```

`lekiwi_teleop`은 다른 워크스페이스 패키지에 의존하지 않으므로 이 패키지만 빌드해도 된다.

테스트는 ROS 노드를 만들지 않고 하드웨어에도 접근하지 않는다. 그래도 다른 장비와 섞이지 않게 로컬 전용으로 돌린다.

```bash
ROS_DOMAIN_ID=78 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --packages-select lekiwi_teleop --event-handlers console_direct+
colcon test-result --verbose
```

## PC ↔ Pi 연결

PC와 Pi의 **모든 터미널**에 같은 값을 쓴다. 도메인 번호는 예시다.

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

| 명령어 (PC) | 확인 내용 |
|---|---|
| `ros2 node list` | `/lekiwi_base_controller_node`, `/lekiwi_motor_driver_node`가 보이는지 |
| `ros2 topic echo --once /lekiwi/motor/ready` | 실물 도구를 켜기 전 `data: false`인지 |
| `ros2 topic info -v /cmd_vel` | 발행자가 없는지 (다른 조작 창 · Nav2 · `ros2 topic pub`을 끈다) |

**Pi 쪽 전제 (실물 도구 공통)**: 베이스 컨트롤러가 `motor_backend: driver`, `enable_motor_write: true`, `torque_enable: false`로 떠 있어야 한다. 실행만으로는 토크가 켜지지 않고, 아래 도구가 `lekiwi/motor/power true`를 불러야 켜진다.

---

### `lekiwi_guarded_base_teleop`

실물 키보드 주행용 조작 창. 키를 누르는 동안만 움직인다.

- 실행 위치: 그래픽 데스크톱이 있는 PC. `python3-tk` 필요
- 시작 순서: zero `cmd_vel` 20회 → `lekiwi/motor/power true` 성공 응답 → 그 뒤에 새로 받은 `lekiwi/motor/ready true` 확인 → 조작 창 활성화
- 서비스 발견은 최대 2 s, 응답은 최대 3 s, ready는 최대 5 s 기다리고, 모두 남은 세션 시간 안에서만 기다린다
- `--yes-i-confirm-motion`이 없으면 시작하지 않는다. 세션 최대 시간은 기본 3600초

```bash
# 현장 확인 직후에만 실행
ros2 run lekiwi_teleop lekiwi_guarded_base_teleop --yes-i-confirm-motion
```

필요하면 `--max-session-duration 3600` 형식으로 세션 최대 시간(초)을 줄인다.

| 키 | 동작 |
|---|---|
| `w` / `s` | 누르는 동안 전진 / 후진 |
| `a` / `d` | 누르는 동안 좌 / 우 이동 |
| `q` / `e` | 누르는 동안 반시계 / 시계 회전 |
| `1` / `2` / `3` | 속도 단계. 전후 · 좌우 `0.04` / `0.05` / `0.06 m/s`, 회전 `0.23` / `0.29` / `0.35 rad/s` |
| Space | 즉시 정지, 속도 단계는 유지 |
| Esc 또는 창 닫기 | zero 명령 반복과 모터 토크 해제 후 종료 (Ctrl+C도 같다) |

**안전 동작**

- 방향 키는 누르는 동안만 움직이고, 놓으면 zero `cmd_vel`을 바로 보낸다. X11 키 자동 반복으로 생기는 놓음 · 누름 쌍(35 ms 이내)은 놓음으로 보지 않는다.
- 창의 포커스를 잃거나 Space를 누르면 정지한다. 그때 누르고 있던 방향 키는 실제로 뗐다가 다시 눌러야 다시 움직인다.
- 여러 방향 키를 함께 누르면 마지막에 누른 한 방향만 쓴다. 방향 전환 중 zero 명령을 먼저 보낸다.
- 속도는 50 ms마다 선속도 `0.005 m/s`, 회전 `0.05 rad/s`씩 목표에 다가간다.
- 실행 중 `lekiwi/motor/ready`가 false가 되면 멈추고 종료한다.
- 정상 종료 · 시간 초과 · 준비 실패 · 예외 모두 zero `cmd_vel` 20회와 `lekiwi/motor/power false`를 시도한다. 정리 실패는 원래 오류를 가리지 않고 함께 보고한다.
- 베이스 컨트롤러 상한이 단계 값보다 낮으면 그 상한으로 잘린다. 3단까지 그대로 쓰려면 베이스 컨트롤러 `max_linear_x`, `max_linear_y`, `max_linear_speed` ≥ `0.06`, `max_angular_z` ≥ `0.35`, `max_wheel_speed` ≥ `1.2`, 모터 드라이버 `speed_tick_limit` ≥ `783`이어야 한다 (저장소 기준 값).

실물 키보드 주행에는 `lekiwi_guarded_base_teleop`만 쓴다. 터미널용 `run_guarded_session`(`r` / `f` 속도 단계)은 모듈과 테스트에 남아 있지만, 실행 파일은 조작 창만 연다.

### `lekiwi_keyboard_teleop`

**실물 키보드 주행에 권장하는 도구.** 창이 키를 누른 순간과 뗀 순간을 따로 받아, **누르고 있는 방향 키를 모두 합친다.**
Pi에서 실행하고 노트북에서 `ssh -X`로 접속하면 창이 노트북 화면에 뜬다. 노트북에 ROS 설정이 필요 없다.

```bash
# 노트북 터미널. -X: Pi 창을 노트북 화면에 띄운다 (Pi sshd X11Forwarding yes, xauth 필요 — lekiwi04는 됨)
ssh -X roboseasy@lekiwi04.local
# Pi (bringup 또는 모터 드라이버 + 베이스 컨트롤러가 다른 터미널에서 실행 중). 현장 확인 직후에만
ros2 run lekiwi_teleop lekiwi_keyboard_teleop --yes-i-confirm-motion
```

| 키 | 동작 (누르고 있는 동안만, **함께 누르면 합쳐서**) |
|---|---|
| `w` / `s` | 앞 / 뒤 |
| `a` / `d` | 왼쪽 / 오른쪽 옆걸음 |
| `q` / `e` | 좌회전(반시계) / 우회전(시계) |
| `w`+`d` 등 | 대각선. 대각선도 같은 속도(그 단계의 m/s)로 간다 |
| `w`+`q` 등 | 앞으로 가며 회전 |
| `r` / `f` | 속도 한 칸 올림 / 내림 (0.01 ~ 0.20 m/s 20단계, 누르고 있으면 0.2 s마다 한 칸). 시작은 0.10 |
| Space | 즉시 정지. 누르고 있던 키는 한 번 뗐다가 다시 눌러야 움직인다 |
| Esc 또는 창 닫기 | 정지 → 토크 끄고 종료 |

- 가속: 50 ms마다 0.02 m/s, 0.12 rad/s씩 → 0.20 m/s까지 0.5 s. 방향 키를 **모두 떼면 바로** zero를 보낸다
- 시작할 때 베이스 컨트롤러 상한(`max_linear_*` 중 최소, `max_angular_z`)을 읽고, 이 도구의 상한 `--max-linear`(기본 `0.10 m/s`) · `--max-angular`(기본 `0.75 rad/s`)과 비교해 작은 쪽 안의 단계까지만 쓴다. 기본으로는 예전과 같이 최고 0.10 m/s다
- 베이스 상한은 2026-10-22부터 0.23 m/s · 1.53 rad/s(텔레오퍼레이션 화면용)다. 이 도구로 더 빠르게 가려면 직접 올린다: `--max-linear 0.2 --max-angular 1.2` (최고 0.20 m/s · 1.16 rad/s)
- 창 밖을 누르면(포커스를 잃으면) 바로 멈춘다. 실행 중 `lekiwi/motor/ready`가 false가 되면 멈추고 종료한다
- 키 반복의 뗌 · 눌림 쌍은 60 ms 안이면 뗀 것으로 보지 않는다 (원격 창에서 쌍이 조금 벌어져 온다)
- 끝날 때(Esc · 창 닫기 · 시간 초과 · 오류) zero `cmd_vel` 20회와 `lekiwi/motor/power false`를 시도한다

조작 창(`lekiwi_guarded_base_teleop`)은 저장소에서 검증한 **한 방향만 · 천천히 가속**하는 보수적인 도구로 그대로 둔다.

### `lekiwi_terminal_teleop`

그래픽 창 없이 **터미널에서** 키보드로 주행한다. PC에서 Pi에 SSH로 접속한 터미널에서 바로 쓸 수 있다.
방향 키 · `1`/`2`/`3` · 가감속은 조작 창과 같고, `r`/`f`로 속도를 한 칸씩 바꿀 수 있다.

```bash
# Pi 터미널 (bringup이 다른 터미널에서 실행 중이어야 한다). 현장 확인 직후에만
ros2 run lekiwi_teleop lekiwi_terminal_teleop --yes-i-confirm-motion
```

| 키 | 동작 (누르고 있는 동안만) |
|---|---|
| `w` / `s` | 앞 / 뒤 |
| `a` / `d` | 왼쪽 / 오른쪽 옆걸음 |
| `q` / `e` | 좌회전(반시계) / 우회전(시계) |
| `r` / `f` | 속도를 한 칸 올림 / 내림. 0.01 ~ 0.20 m/s, 0.01 m/s 간격 20단계 (회전 0.06 ~ 1.16 rad/s가 함께 바뀐다). 누르고 있으면 0.2 s마다 한 칸. **베이스 컨트롤러 상한을 넘는 단계는 쓰지 않는다** |
| `1` / `2` / `3` | 속도 바로 고르기: `0.04` / `0.05` / `0.06 m/s` (회전 `0.23` / `0.29` / `0.35 rad/s`). 시작은 `0.04` |
| Space | 즉시 정지. 누르고 있던 키는 한 번 뗐다가 다시 눌러야 움직인다 |
| Esc 또는 Ctrl+C | 정지 → 토크 끄고 종료 |

**"누르고 있음"을 판단하는 방법**

터미널은 키를 뗀 순간을 알려 주지 않는다. 키를 누르고 있으면 터미널이 같은 글자를 반복해서 보내므로,
그 반복 입력이 `--release-timeout`(기본 `0.6 s`) 동안 끊기면 뗀 것으로 보고 zero `cmd_vel`을 보낸다.

- 키를 **톡 치면** 약 0.6 s 움직이고 멈춘다 (속도 1단이면 2 cm 남짓)
- 키를 **누르고 있으면** 계속 움직인다. 처음 0.5 s는 키보드 반복이 시작되기 전이지만 0.6 s 안이라 끊기지 않는다
- 키를 **떼면** 0.6 s 안에 멈춘다 (조작 창보다 반응이 느리다). 0.10 m/s에서는 떼고 나서 최대 6 cm, 0.20 m/s에서는 12 cm쯤 더 간다
- 시작할 때 `/lekiwi_base_controller_node`의 `max_linear_x` · `max_linear_y` · `max_linear_speed` 중 가장 작은 값과 `max_angular_z`를 읽고, 이 도구의 상한 `--max-linear`(기본 `0.10 m/s`) · `--max-angular`(기본 `0.75 rad/s`)과 비교해 작은 쪽 안의 단계까지만 쓴다. 기본으로는 예전과 같이 최고 0.10 m/s다 (베이스 상한은 2026-10-22부터 0.23 m/s · 1.53 rad/s). 상태 줄에 `(최고 0.10)`처럼 나온다. 상한을 읽지 못하면 토크를 켜지 않고 끝난다
- 더 빠르게 가려면 직접 올린다: `--max-linear 0.2 --max-angular 1.2`면 20단계를 모두 쓴다 (최고 0.20 m/s · 1.16 rad/s)
- 속도를 바꾸면 지금 속도에서 50 ms마다 0.005 m/s씩 새 속도로 다가간다 (갑자기 튀지 않는다)
- 키보드 반복 시작 지연을 0.6 s보다 길게 바꿔 두었다면 `--release-timeout`을 그보다 길게 준다 (`0.2` ~ `1.5`)

**안전 동작**

- 시작 순서: zero `cmd_vel` 20회 → `lekiwi/motor/power true` → 그 뒤에 새로 받은 `lekiwi/motor/ready true` 확인 → 키 입력 시작
- 50 ms마다 `cmd_vel`을 보낸다. 실행 중 `lekiwi/motor/ready`가 false가 되면 멈추고 종료한다
- 정상 종료 · Ctrl+C · 시간 초과(`--max-session-duration`, 기본 3600 s) · SSH 끊김(SIGHUP) · 오류 모두 zero `cmd_vel` 20회와 `lekiwi/motor/power false`를 시도한다
- 이 프로세스가 강제로 죽어도 베이스 컨트롤러가 0.3 s 안에 멈추고, 모터 드라이버가 3 s 안에 토크를 끈다
- 방향 키(화살표)는 무시한다. Esc 하나만 종료로 본다

`lekiwi_base_teleop`(mock 전용)과 달리 실물용이다. 표준 `teleop_twist_keyboard`도 쓸 수는 있지만 토크를 켜고 끄지 않고,
키를 누를 때만 명령을 보내 처음 누를 때 0.3 s timeout에 걸려 한 번 멈칫한다. 실물에는 이 도구를 쓴다.

### `lekiwi_safe_cmd_vel`

정해진 횟수만 `cmd_vel`을 발행하는 짧은 실물 확인용 도구. 끝없이 남을 수 있는 `ros2 topic pub --rate` 대신 쓴다.

- 실행 위치: PC 터미널
- 기본 상한: `--max-linear 0.05` (평면 속도 m/s), `--max-angular 0.3` (rad/s), `--max-duration 2.0` (s), `--max-rate 50.0` (Hz). 넘으면 시작하지 않는다. 더 크게 하려면 현장 확인 뒤 해당 `--max-*`를 명시적으로 올린다
- non-zero 명령이나 `--arm-motors`는 `--yes-i-confirm-motion`이 없으면 거부한다
- `cmd_vel` 구독자가 `--min-subscribers`(기본 1)개 보일 때까지 최대 `--discovery-timeout`(기본 8 s) 기다린다
- 순서: zero `--zero-cycles`(기본 20)회 → (`--arm-motors`) `lekiwi/motor/ready false` 확인(2 s) → `lekiwi/motor/power true` → `lekiwi/motor/ready true` 확인(2 s) → 명령 `ceil(duration × rate)`회 → zero → (`--arm-motors`) `lekiwi/motor/power false`
- 명령 도중 예외가 나도 zero와 `lekiwi/motor/power false`를 시도한다

```bash
# zero만 보내기 (확인 옵션 불필요)
ros2 run lekiwi_teleop lekiwi_safe_cmd_vel

# 예: 바퀴를 띄운 상태에서 전진 0.03 m/s × 1 s. 현장 확인 직후에만
ros2 run lekiwi_teleop lekiwi_safe_cmd_vel --arm-motors --yes-i-confirm-motion \
  --linear-x 0.03 --duration 1.0
```

### `lekiwi_nav2_motor_guard`

Nav2 주행 동안에만 바퀴 토크를 켜 두는 감독 프로세스. 보통 Nav2 launch가 실행한다.

- 실행 위치: Nav2와 같은 컴퓨터 (Pi)
- 자신은 주행 명령을 보내지 않는다. 토크를 켜기 전과 종료할 때 zero `cmd_vel` 20회만 보낸다
- 토크를 켜기 전 확인 (`--startup-timeout`, 기본 5 s):
  1. `lekiwi/sensor/scan_filtered`와 `odom`을 최근 1 s 안에 받았는지
  2. `lekiwi/motor/ready`가 false인지
  3. `cmd_vel` 발행자가 `/collision_monitor` 하나, 구독자가 `/lekiwi_base_controller_node` 하나인지
  4. 베이스 컨트롤러 파라미터: `motor_backend == driver`, `enable_motor_write == true`, `odom_source == encoder`, `torque_enable == false`
  5. zero를 보낸 뒤 그래프가 다시 3번과 같아지는지(최대 2 s), 센서가 여전히 최신인지
  6. `--operator-nodes`(기본 `rviz,rviz2`) 이름의 노드(조작 화면)가 ROS 그래프에 있는지. 이름만 비교하고 네임스페이스 · 컴퓨터는 보지 않는다
- 확인을 통과하면 `lekiwi/motor/power true` → 5 s 안에 `lekiwi/motor/ready true`
- 주행 중: ready가 false가 되거나, 스캔 · odom이 1 s 넘게 끊기거나, `cmd_vel` 그래프가 바뀌거나, 조작 화면 노드가 모두 사라지면(0.5 s마다 확인) 토크를 끈다. `--max-session-duration`(기본 600 s)이 지나도 끈다
- 목표에 도착해도 토크는 켜진 채로 남는다. RViz를 닫거나 launch를 멈추면 끈다
- 다른 컴퓨터의 RViz가 Wi-Fi 끊김 · 강제 종료로 사라지면 DDS가 그 노드를 지우는 데 최대 약 20 s(Fast DDS 기본) 걸린다. 정상적으로 닫으면 바로 사라진다
- 같은 `ROS_DOMAIN_ID`의 RViz는 어느 것이든 조작 화면으로 친다 (Cartographer · 모델 미리보기 RViz도 이름이 `rviz2`). Nav2 때는 RViz를 하나만 연다
- 한 번 끄면 다시 켜지 않는다. RViz를 60 s 안에 열지 않았거나 닫았다면 Nav2 launch를 다시 시작한다
- 종료: `lekiwi/motor/power false` → zero 20회. 끄기 응답이 없으면 종료 코드 1
- `--yes-i-confirm-motion` 필수. launch가 붙이는 `--ros-args ...`는 받아들이지만, 그것으로 필수 · 검증 옵션을 건너뛸 수는 없다

```bash
# 보통은 Nav2 launch가 실행한다. 직접 실행 예
ros2 run lekiwi_teleop lekiwi_nav2_motor_guard --yes-i-confirm-motion \
  --startup-timeout 60 --max-session-duration 3600     # --operator-nodes 기본 rviz,rviz2
```

### `lekiwi_distance_trial`

짧은 직선 이동 한 번의 엔코더 odom 변위와 나중에 잰 실측값을 비교한다. 기본은 전진 0.02 m/s × 10 s (0.2 m).
**odom이 목표에 닿을 때까지 움직이는 제어기가 아니다.** 정해진 시간만큼 명령을 보내고 멈춘다.

- 실행 위치: PC 터미널. `plan` · `compare`는 ROS 없이 동작한다
- 상한: 거리 ≤ 0.3 m, 속도 ≤ 0.03 m/s, 시간 0.5 ~ 30 s. 방향 `forward`, `backward`, `left`, `right`
- odom을 초기화하거나 보정값을 바꾸지 않는다. 컨트롤러 · 센서를 새로 실행하지 않는다
- `run` 사전 조건 (하나라도 다르면 토크를 켜지 않는다):
  - 베이스 컨트롤러: `motor_backend: driver`, `enable_motor_write: true`, `torque_enable: false`, `odom_source: encoder`, `encoder_feedback_source: position`, `use_sim_time: false`, `0.15 < cmd_vel_timeout <= 0.5`
  - 모터 드라이버: `speed_tick_scale`, `speed_tick_limit` (바퀴 속도 상한 계산에 쓴다)
  - 요청 속도가 `max_linear_x` / `max_linear_y` / `max_linear_speed`와 `min(max_wheel_speed, speed_tick_limit / speed_tick_scale)`에 잘리지 않을 것
  - `cmd_vel` 발행자는 이 도구 하나, 구독자는 `/lekiwi_base_controller_node` 하나. `odom` · `lekiwi/motor/ready` 발행자도 `/lekiwi_base_controller_node` 하나
  - `lekiwi/motor/ready false`, odom 1 s 정지 구간
- 중단 조건: odom 누락 · 0.3 s 넘는 간격 · 시간 역행 · 5 cm 넘는 위치 점프 · 0.2 rad 넘는 yaw 점프, ready 해제, publisher 충돌, 명령 간격 0.15 s 초과, 횡방향 3 cm · 회전 0.2 rad · 목표 + 5 cm 초과
- 끝 · 중단 · Ctrl+C · SIGTERM · SIGHUP: zero 20회 → `lekiwi/motor/power false` → `lekiwi/motor/ready false` 확인. 자동 재시도는 없다
- 결과 JSON은 새 파일로만 쓴다 (기존 파일을 덮어쓰지 않는다). `completed`는 절차 완료이지 정확도 합격이 아니다. `physical_stop_confirmed`, `torque_off_register_verified`는 자동으로 참이 되지 않는다

```bash
ros2 run lekiwi_teleop lekiwi_distance_trial plan

# 바닥에서, 현장 확인 직후 한 번만
ros2 run lekiwi_teleop lekiwi_distance_trial run \
  --yes-i-confirm-motion --output /tmp/distance-forward-01.json

# 시작 방향으로 투영한 실측 이동량 (예: 0.18 m). 반대로 움직였으면 음수
ros2 run lekiwi_teleop lekiwi_distance_trial compare \
  --report /tmp/distance-forward-01.json --measured 0.18 \
  --tolerance 0.01 --output /tmp/distance-forward-01-comparison.json
```

`--mock`은 베이스 컨트롤러가 `motor_backend: mock`, `enable_motor_write: false`, `odom_source: command`일 때만 실행하고 토크를 켜지 않는다.
mock 결과는 `compare`에 쓸 수 없다. mock에서도 `speed_tick_*`를 모터 드라이버 노드에서 읽으므로 드라이버 노드가 떠 있어야 한다.

### `lekiwi_odom_calibration`

바닥 주행 실측값으로 `encoder_odom_scale` 후보를 계산한다. ROS에 연결하지 않고 모터를 건드리지 않는다.
계산식은 `현재 scale × 실측 / |odom|`이다.

```bash
ros2 run lekiwi_teleop lekiwi_odom_calibration \
  --current-scale 1.0 --odom-delta 0.60 --measured-delta 0.50
```

출력의 `encoder_odom_scale:=...`를 베이스 컨트롤러 파라미터로 넣고 다시 검증한다.
시험 사이에 odom을 원점으로 되돌릴 때는 베이스 컨트롤러 서비스를 부른다.

```bash
ros2 service call /lekiwi/motor/reset_odometry std_srvs/srv/Trigger {}
```

### `lekiwi_base_teleop`

터미널 키 입력마다 `cmd_vel`을 보내는 **mock 전용 데모**. 모터 전원 · ready 확인이 없고 실행 시간 제한도 없다.
`motor_backend: driver` 또는 `enable_motor_write: true`인 베이스와 함께 실행하지 않는다.

| 키 | `cmd_vel` |
|---|---|
| `w` / `s` | 전진 / 후진 (`linear.x`) |
| `a` / `d` | 좌 / 우 이동 (`linear.y`) |
| `q` / `e` | 반시계 / 시계 회전 (`angular.z`) |
| `+`, `=` / `-`, `_` | 선속도 · 각속도 증가 / 감소. `cmd_vel`은 보내지 않는다 |
| `space` | zero |
| `x`, `Esc`, `Ctrl-C` | zero 3회 후 종료 |

```bash
ros2 run lekiwi_teleop lekiwi_base_teleop --ros-args \
  -p max_linear_speed:=0.10 -p max_angular_speed:=0.3 -p speed_scale_step:=1.2
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `cmd_vel_topic` | `cmd_vel` | 발행 토픽 |
| `linear_speed`, `angular_speed` | `0.05`, `0.3` | 시작 속도 [m/s], [rad/s] |
| `min_linear_speed`, `max_linear_speed` | `0.02`, `0.10` | 선속도 범위 |
| `min_angular_speed`, `max_angular_speed` | `0.2`, `0.3` | 각속도 범위 |
| `speed_scale_step` | `1.2` | `+` / `-` 한 번의 배율 |

---

## 안전 처리 계층

| 상황 | 처리하는 곳 | 결과 |
|---|---|---|
| 도구가 정상 종료 · 시간 초과 · 예외로 끝남 | 이 패키지 도구 | zero `cmd_vel` 반복 → `lekiwi/motor/power false` |
| 도구 프로세스 강제 종료, PC ↔ Pi 통신 끊김 | 베이스 컨트롤러 | `cmd_vel`이 0.3 s 넘게 끊기면 바퀴 명령 0 |
| 베이스 컨트롤러가 멈춤 | 모터 드라이버 | 0.5 s 뒤 속도 0, 3 s 뒤 바퀴 토크 끔 |

자세한 계층은 [`docs/interfaces.md`](../docs/interfaces.md)의 "안전 처리 계층"을 따른다.

## 알려진 한계

- 이 워크스페이스의 베이스 컨트롤러 · 쓰기 가능한 모터 드라이버와 함께 실행해 보지 않았다. 파라미터 이름이 다르면 `lekiwi_distance_trial`은 `Missing ... parameters`로, Nav2 감독은 파라미터 확인 단계에서 멈추고 토크를 켜지 않는다.
- namespace를 준 실행은 해 보지 않았다.
- 토크 해제는 서비스 응답과 `lekiwi/motor/ready false`로만 확인한다. 모터 레지스터를 직접 읽지 않는다.
- `lekiwi_distance_trial`의 ROS 노드 이름은 `lekiwi_guarded_base_teleop`이다 (원본과 같다). 두 도구를 동시에 쓰지 않는다.

## 문제 해결

| 증상 | 원인 · 조치 |
|---|---|
| `Package 'lekiwi_teleop' not found` | PC 워크스페이스를 source하지 않음. `sb` |
| `No module named 'tkinter'` | `sudo apt install python3-tk` |
| `lekiwi/motor/power service unavailable after 2s` | Pi 베이스 컨트롤러가 꺼져 있거나 `ROS_DOMAIN_ID` · `ROS_AUTOMATIC_DISCOVERY_RANGE`가 다름 |
| `lekiwi/motor/ready did not produce a true sample` | 엔코더(`lekiwi/motor/states`)가 들어오지 않음. 모터 전원 · 드라이버 확인 |
| `expected only /collision_monitor on cmd_vel` | 다른 `cmd_vel` 발행자가 있음. `ros2 topic info -v /cmd_vel` |
| `base is not using the writable 'driver' motor backend` | 베이스 컨트롤러가 mock이거나 쓰기가 꺼져 있음 |
| `Missing /lekiwi_motor_driver_node parameters` | 모터 드라이버가 꺼져 있거나 `speed_tick_scale` · `speed_tick_limit`를 선언하지 않음 |

---

## 패키지 구조

```text
lekiwi_teleop/
├── package.xml
├── setup.py · setup.cfg
├── README.md
├── resource/lekiwi_teleop
├── lekiwi_teleop/
│   ├── interface_names.py      # 노드 · 토픽 · 서비스 이름 (docs/interfaces.md)
│   ├── motor_session.py        # 모터 전원 서비스 · ready 구독 공통 코드
│   ├── guarded_base_teleop.py  # lekiwi_guarded_base_teleop
│   ├── safe_cmd_vel.py         # lekiwi_safe_cmd_vel
│   ├── nav2_motor_guard.py     # lekiwi_nav2_motor_guard
│   ├── distance_trial.py       # lekiwi_distance_trial (계획 · 비교, ROS 없음)
│   ├── distance_trial_ros.py   # lekiwi_distance_trial run의 ROS 연결
│   ├── odom_calibration.py     # lekiwi_odom_calibration
│   └── base_teleop.py          # lekiwi_base_teleop (mock 전용)
└── test/                       # 하드웨어 · ROS 네트워크 없이 도는 단위 테스트
```

## 참고한 코드

- **LeKiwi 저장소 `feature/lekiwi-ros2`의 `lekiwi_teleop`**: 이식 원본. 안전 순서 · 상한 · 테스트를 그대로 가져왔다
- **LeKiwi 저장소 `docs/hardware/base_operation.md`, `distance_trial.md`**: 실행 방법과 안전 설명
- **이 워크스페이스 `docs/interfaces.md`**: 노드 · 토픽 · 서비스 이름

## 검증 기록

- 2026-10-18, PC (Ubuntu 24.04 · Jazzy): 빌드 성공. `colcon test` (pytest) 125개 통과, 실패 0 (junit 기준 subTest 포함 176개)
- 2026-10-18, PC 로컬 전용(`ROS_DOMAIN_ID=78`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)에서 **가짜** `lekiwi_base_controller_node` · `lekiwi_motor_driver_node` · `collision_monitor`를 띄워 이름 연결만 확인했다. 실물 로봇 · 모터는 쓰지 않았다
  - `lekiwi_distance_trial run --distance 0.02 --speed 0.02`: `speed_tick_*`를 모터 드라이버 노드에서 읽고, 전원 켜기 → 1.0 s 명령 → 전원 끄기 순서로 `completed: true`
  - `lekiwi_nav2_motor_guard --max-session-duration 3`: 그래프 · 파라미터 확인 → 전원 켜기 → 시간 제한 → 전원 끄기, 종료 코드 0. `motor_backend: feetech`이면 전원을 켜지 않고 종료 코드 1
  - `lekiwi_safe_cmd_vel --arm-motors --linear-x 0.03 --duration 0.5`: 전원 켜기 → 명령 → 전원 끄기, 종료 코드 0
- 조작 창(`lekiwi_guarded_base_teleop`)의 실제 키 입력, 새 베이스 컨트롤러 · 모터 드라이버와의 실행, 실물 주행은 아직 확인하지 않았다
