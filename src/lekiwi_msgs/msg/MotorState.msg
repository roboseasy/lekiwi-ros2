# Feetech STS3215 모터 한 개의 상태 (lekiwi_motor_driver가 발행)

# status: Status 레지스터(65) 비트. 0이면 정상
uint8 STATUS_VOLTAGE = 1
uint8 STATUS_ANGLE_SENSOR = 2
uint8 STATUS_OVERHEAT = 4
uint8 STATUS_OVERCURRENT = 8
uint8 STATUS_OVERLOAD = 32

# mode: Operating_Mode 레지스터(33)
uint8 MODE_POSITION = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_PWM = 2
uint8 MODE_STEP = 3
uint8 MODE_UNKNOWN = 255

uint8 id
string name
bool online               # 이번 주기 읽기 성공. false면 position ~ status 값은 0

int32 position_ticks      # 0~4095 (한 바퀴 4096)
float64 position          # [rad] (position_ticks - 2048) * 2pi / 4096. 로봇 모델의 관절 각도가 아니다
float64 velocity          # [rad/s]
float64 load              # [%] 모터 구동 출력 비율. 부호는 방향. 토크 측정값이 아니다
float64 voltage           # [V]
float64 temperature       # [degC]
float64 current           # [A]
bool moving
uint8 status              # STATUS_* 비트 조합

# 느린 주기(slow_read_period, 기본 1초)마다 갱신. 응답이 끊기면 MODE_UNKNOWN, false
uint8 mode                # MODE_*
bool torque_enabled
