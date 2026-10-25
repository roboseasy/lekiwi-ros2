# lekiwi_msgs

LeKiwi 온디바이스 패키지가 함께 쓰는 **커스텀 msg/srv/action 모음**.
기능 패키지 안에 메시지를 두지 않고 여기에 모은다. 토픽 이름은 [이름 계약](../docs/interfaces.md)을 따른다.

## 메시지

| 메시지 | 쓰는 곳 | 내용 |
|--------|---------|------|
| `MotorState` | `lekiwi_motor_driver` | Feetech STS3215 모터 한 개의 위치 · 속도 · 부하 · 전압 · 온도 · 전류 · 상태 |
| `MotorStateArray` | `lekiwi_motor_driver` → `/lekiwi/motor/states` | 버스에 있는 모든 모터의 `MotorState` |
| `WheelCommand` | `lekiwi_base_controller` → `/lekiwi/motor/wheel_command` → `lekiwi_motor_driver` | 바퀴 모터 ID별 속도 명령 [rad/s] |
| `ArmCommand` | `lekiwi_lerobot` follower_arm → `/lekiwi/motor/arm_command` → `lekiwi_motor_driver` | 팔 모터 ID별 목표 위치 [tick, 0~4095] |
| `ArmRegisters` | `lekiwi_motor_driver` → `/lekiwi/motor/arm_registers` → `lekiwi_lerobot` follower_arm | 팔 모터 EEPROM 값 (Homing_Offset · 위치 한계 · P/D/I). 보정 JSON과 비교용, 읽기만 |
| `LerobotState` | `lekiwi_lerobot` follower_arm → `/lekiwi/lerobot/state` → PC 데이터셋 기록기 | lerobot LeKiwi `observation.state` 9개 (`names` · `values`: lerobot 이름 · 순서 · 단위) |

필드와 단위는 `msg/*.msg` 파일의 주석을 본다.

```bash
ros2 interface show lekiwi_msgs/msg/MotorState
```

## 빌드

메시지를 쓰는 패키지보다 먼저 빌드한다.

```bash
cd ~/lekiwi_ws
cbs lekiwi_msgs && sb
```

메시지를 바꾸면 이 패키지와, 이 메시지를 쓰는 패키지를 모두 다시 빌드한다.

## 새 메시지를 추가할 때

1. `msg/` (또는 `srv/`, `action/`)에 파일을 만든다. 이름은 `CamelCase`, 필드는 `snake_case`
2. `CMakeLists.txt`의 `rosidl_generate_interfaces`에 파일을 추가한다
3. 다른 패키지의 메시지를 쓰면 `DEPENDENCIES`와 `package.xml`의 `<depend>`에 추가한다
4. 위 표에 한 줄 추가한다
