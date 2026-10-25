# lekiwi_description

LeKiwi **로봇 모델(URDF · 메시)** 패키지. Pi의 `robot_state_publisher`와 PC의 RViz가 함께 쓴다.
LeKiwi 저장소(`feature/lekiwi-ros2`)의 같은 패키지를 옮겨 왔다. 이름 규칙은 [이름 계약](../docs/interfaces.md)을 따른다.

- 진입점: `urdf/lekiwi.urdf.xacro` (교육용 몸체 + SO101 팔 + 전방 · 손목 카메라)
- `use_hardware_sensors:=true`면 실물 라이다(`lidar_link`) · IMU(`imu_link`) 프레임을 붙인다. `lekiwi_bringup`이 이렇게 쓴다
- 팔은 모델만 있다. 팔 제어는 포함하지 않는다

## TF 트리

```text
base_footprint ─(z 0.075)→ base_link ─┬→ {left,back,right}_wheel_mount_link → {left,back,right}_wheel_link  (continuous)
                                       ├→ front_camera_optical_frame
                                       └→ soarm_base_link ─┬→ lidar_link   (z -0.1025, 뒤집힌 장착 roll π)
                                                           ├→ imu_link     (z -0.0525, yaw π/2 임시값)
                                                           └→ 팔 관절 6개 … wrist_link → wrist_camera_optical_frame
```

(2026-10-18 lekiwi04에서 `xacro … use_hardware_sensors:=true`로 만든 모델 기준)

`odom → base_footprint`는 베이스 컨트롤러가, `map → odom`은 Cartographer 또는 AMCL이 낸다.

## 설정 파일

| 파일 | 내용 |
|------|------|
| `config/base_frame.yaml` | 옛 모터 기구학 원점의 자세(`kinematics_frame_x/y/yaw`). `lekiwi_bringup`이 베이스 컨트롤러에 넣는다. URDF는 읽지 않는다 |
| `config/camera_mounts.json` | 카메라 장착 자세 출처. `calibrated: false` (실측 보정 전) |
| `rviz/base.rviz` | 실행 중인 로봇 보기: 고정 프레임 `odom`, 로봇 모델 · TF · `/odom` · `/lekiwi/sensor/scan_filtered` |
| `rviz/model.rviz` | 모델 미리보기 |

## 실행

모델 미리보기 (PC, 로봇 없이):

```bash
ros2 launch lekiwi_description display.launch.py
```

실행 중인 로봇 보기는 `ros2 launch lekiwi_bringup rviz.launch.py`를 쓴다.

필요한 apt 패키지: `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-joint-state-publisher`, (PC) `ros-jazzy-rviz2`.

## 시험

```bash
colcon test --packages-select lekiwi_description --event-handlers console_direct+
```

`test_default_model.py`는 `xacro`로 모델을 만들어 프레임 트리 · 메시 경로 · 원본 메시 해시 · 바퀴 위치와 `base_frame.yaml`의 일치 등을 확인한다.

## 출처와 기록

- 모델 출처와 가져온 범위: `docs/IMPORT.md`, `docs/SOURCE.md`, `docs/import_manifest.json`
- 좌표계 변경 근거: `docs/FRAME_MIGRATION.md` (저장소 시절 기록이라 노드 이름 `lekiwi_node`가 나온다. 지금은 `lekiwi_base_controller_node`)
- 새 좌표계의 실제 모터 · 센서 방향은 현장 검증 전이다. IMU yaw π/2는 임시값이다

## 옮기면서 바꾼 것 (2026-10-18)

- `config/base_frame.yaml`의 노드 키: `lekiwi_node` → `lekiwi_base_controller_node`
- `rviz/base.rviz`의 스캔 토픽: `/scan` → `/lekiwi/sensor/scan_filtered`
