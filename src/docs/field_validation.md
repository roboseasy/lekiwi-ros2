# 현장 확인 절차 (실물)

온디바이스 워크스페이스를 실물로 확인하는 순서다. 단계마다 **통과 기준**을 넘어야 다음 단계로 간다.
1단계는 로봇이 움직이지 않는다. **2단계부터는 바퀴가 실제로 돈다.**

> ⚠️ 2단계부터: 시작 전에 주변에 사람 · 케이블 · 장애물이 없는지, 로봇에서 손을 뗐는지,
> **즉시 주 전원을 끌 사람이 옆에 있는지** 현장에서 확인한다. 서비스 성공 응답은 물리 정지를 뜻하지 않는다.
> 바퀴가 멈췄는지 눈으로 확인한다.

새 좌표계에서 바퀴 ID · 방향과 센서 방향은 아직 실물 검증 전이다. 결과는 각 패키지 README의 "검증 기록"에 날짜와 함께 남긴다.

---

## 0. 준비

### Pi (한 번만)

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher   # lekiwi04는 2026-10-18 설치함
cd ~/lekiwi_ws && cb && sb
cp ~/lekiwi_ws/src/lekiwi_bringup/config/robot_profile.yaml ~/lekiwi_ws/robot_profile.yaml
nano ~/lekiwi_ws/robot_profile.yaml     # 포트(by-id 권장), 바퀴 ID · 방향, IMU 주소
```

같은 모터 버스를 쓰는 프로그램(저장소 `lekiwi_node` 브링업, LeRobot 호스트)이 꺼져 있는지 확인한다.

```bash
fuser -v /dev/ttyACM0      # 아무것도 나오지 않아야 한다
```

Cartographer · Nav2 (4 · 5단계, Pi에서 실행): [`lekiwi_cartographer` 설치](../lekiwi_cartographer/README.md#설치-pi-한-번만)의 apt 명령. lekiwi04는 2026-10-18 설치함.

### PC (한 번만)

PC는 RViz로 보기만 한다. Cartographer · Nav2는 설치하지 않는다.

```bash
sudo apt install ros-dev-tools ros-jazzy-nav2-rviz-plugins     # Nav2 Goal 도구 · 입자 표시
cd $LEKIWI_WS/src && git pull       # 최신 커밋 (LEKIWI_WS: 저장소 README 1-②)
cd .. && colcon build --symlink-install --packages-select lekiwi_description && source install/setup.bash
```

### 모든 터미널 (PC · Pi)

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ros2 node list`가 비어 보이면 `ros2 daemon stop` 후 다시 실행한다 (예전 설정으로 뜬 데몬).

---

## 1단계: 모델 · 센서 · 모터 읽기 (움직이지 않음)

**Pi**

```bash
ros2 launch lekiwi_bringup bringup.launch.py use_base_controller:=false
```

**PC**

```bash
ros2 launch lekiwi_bringup rviz.launch.py
ros2 topic hz /lekiwi/sensor/scan_filtered       # 약 10 Hz
ros2 topic hz /lekiwi/sensor/offset_imu          # 약 100 Hz
ros2 topic hz /lekiwi/motor/states               # 약 30 Hz
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

**통과 기준**

- [ ] RViz에 로봇 모델이 보이고, 스캔 점이 실제 벽 · 물체와 같은 방향 · 거리에 찍힌다 (로봇 앞에 상자를 두고 확인)
- [ ] 로봇을 손으로 반시계로 살짝 돌리면 `/lekiwi/sensor/offset_imu`의 `angular_velocity.z`가 양수
- [ ] `/lekiwi/motor/states`에서 9개 모터 모두 `online: true`, `status: 0`, 바퀴 `mode: 1`, `torque_enabled: false`
- [ ] (카메라를 쓸 때) `use_camera:=true`로 켜고 영상 `frame_id`가 `front_camera_optical_frame`, `wrist_camera_optical_frame`

2026-10-18 lekiwi04에서 확인한 것 (사람 없이, 움직임 없음): 노드 6개 기동, 모터 9/9 응답, 스캔 · 걸러진 스캔 10 Hz,
IMU 100 Hz, 모터 상태 20 Hz, `joint_states` 50 Hz, TF `base_footprint → lidar_link` = (-0.001, 0.002, 0.025) m.
**RViz에서 실제 물체와 방향이 맞는지는 아직 사람이 보지 않았다.**

---

## 2단계: 바퀴 띄운 저속 시험 ⚠️ 바퀴가 돈다

로봇을 받침대에 올려 **바퀴가 바닥에 닿지 않게** 한다.

**Pi**

```bash
ros2 launch lekiwi_bringup bringup.launch.py
```

로그에 `startup: wheel speed zeroed and torque disabled`와 `wheel write ready for ids [7, 8, 9]`가 나와야 한다.

**PC**

```bash
ros2 topic echo --once /lekiwi/motor/ready        # data: false
# 전진 0.03 m/s × 1 s. 끝나면 스스로 토크를 끈다
ros2 run lekiwi_teleop lekiwi_safe_cmd_vel --arm-motors --yes-i-confirm-motion --linear-x 0.03 --duration 1.0
```

같은 방법으로 `--linear-y 0.03`(왼쪽), `--angular-z 0.2`(반시계)를 한 번씩 본다.

**통과 기준**

- [ ] 전진 명령에 세 바퀴가 "로봇이 앞으로 갈 방향"으로 돈다. 왼쪽 · 반시계도 같다. 틀리면 `robot_profile.yaml`의 `wheel_ids` · `wheel_directions`를 고친다
- [ ] 명령이 끝나면 0.3 s 안에 바퀴가 멈춘다
- [ ] 끝나면 `ros2 topic echo --once /lekiwi/motor/states`에서 바퀴 `torque_enabled: false`
- [ ] 바퀴가 도는 동안 Pi에서 베이스 컨트롤러만 강제 종료하면(`pkill -9 -f lekiwi_base_controller_node`) 드라이버가 0.5 s 안에 바퀴를 세우고 3 s 안에 토크를 끈다

---

## 3단계: 바닥 20 cm 주행 ⚠️

바닥에 내려놓고 줄자를 바닥에 붙인다.

```bash
ros2 run lekiwi_teleop lekiwi_distance_trial plan                       # 계획만 출력
ros2 run lekiwi_teleop lekiwi_distance_trial run --yes-i-confirm-motion --output ~/trial_01.json
ros2 run lekiwi_teleop lekiwi_distance_trial compare --report ~/trial_01.json \
  --measured <줄자로 잰 m> --output ~/trial_01_compare.json
```

정확한 인자는 `lekiwi_teleop/README.md`를 본다. 키보드 주행은 `ros2 run lekiwi_teleop lekiwi_guarded_base_teleop --yes-i-confirm-motion`.

**통과 기준**

- [ ] odom 거리와 줄자 거리 차이를 기록하고, 필요하면 `lekiwi_odom_calibration`으로 `encoder_odom_scale` 후보를 계산해 프로파일에 넣는다
- [ ] 키보드 창에서 키를 떼거나 창 밖을 누르면 바로 멈춘다

---

## 4단계: 지도 작성 ⚠️

**Pi 터미널 1**: 2단계와 같은 bringup. 나머지:

```bash
# Pi 터미널 2: 지도 작성
ros2 launch lekiwi_cartographer cartographer.launch.py
# PC: 지도 보기
rviz2 -d $LEKIWI_WS/src/lekiwi_cartographer/rviz/slam.rviz
# 노트북 → Pi (ssh -X): 천천히 주행
ros2 run lekiwi_teleop lekiwi_keyboard_teleop --yes-i-confirm-motion
# Pi 새 터미널: 로봇을 세운 뒤 저장 (~/maps/lekiwi/)
ros2 run lekiwi_cartographer save_map.sh
```

**통과 기준**

- [ ] 방 하나를 천천히 돌고 지도가 겹치지 않는다. 저장 파일(`~/maps/lekiwi/`)이 생긴다
- [ ] Cartographer가 IMU를 받는지 확인: 로그에 IMU 관련 경고가 없고, `ros2 topic info -v /lekiwi/sensor/offset_imu`의 구독자에 Cartographer가 보인다 (QoS 불일치면 구독자가 연결되지 않는다)
- [ ] 이 단계 동안 Pi에서 `top`으로 CPU를 기록한다. 모터 · 센서 노드와 함께 돌아도 여유가 있는지 보는 근거다

---

## 5단계: Nav2 ⚠️

```bash
# PC: RViz를 먼저 연다 (감독 도구는 이 화면이 보여야 토크를 켠다)
rviz2 -d $LEKIWI_WS/src/lekiwi_navigation2/rviz/nav2.rviz
# Pi 터미널 2 (Cartographer · 키보드 창은 끈 상태)
ros2 launch lekiwi_navigation2 navigation.launch.py map:=$HOME/maps/lekiwi/map.yaml
```

RViz에서 초기 위치를 주고 가까운 목표 하나를 준다.

**통과 기준**

- [ ] 목표에 한 번 도착한다
- [ ] 경로 앞에 상자를 놓으면 충돌 감시가 멈춘다
- [ ] PC RViz를 닫으면 감독 도구가 토크를 끈다 (Pi 터미널에 `operator view closed`)
- [ ] 이 단계 동안 Pi에서 `top`으로 CPU를 기록한다

---

## 문제가 생기면

| 증상 | 확인 |
|------|------|
| `lekiwi/motor/power`가 `refused: wheel N ... not velocity mode` | 바퀴가 속도 모드가 아니다. LeRobot 설정 도구로 맞춘다 |
| `power true`가 `disable motor power before recovering a latched fault` | 고장이 고정됨. 원인을 확인하고 `ros2 service call /lekiwi/motor/power std_srvs/srv/SetBool "{data: false}"` 후 다시 |
| `ready`가 true가 되지 않음 | `/lekiwi/motor/states`에서 바퀴가 `online`인지, 드라이버 로그 확인 |
| 바퀴 방향이 반대 | `robot_profile.yaml`의 `wheel_directions` (또는 ID 순서) |
| 노드는 보이는데 토픽 데이터가 안 옴 | 두 장비의 `ROS_DOMAIN_ID` · `ROS_AUTOMATIC_DISCOVERY_RANGE`가 같은지, `ros2 daemon stop` |
