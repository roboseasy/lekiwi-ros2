# 바퀴 모터 속도 명령 (lekiwi_base_controller → lekiwi_motor_driver)
#
# 드라이버는 이 메시지를 받으면 바로 SYNC_WRITE로 Goal_Speed(46)에 쓴다.
# 드라이버 설정의 wheel_ids에 없는 ID(예: 팔 1~6)가 하나라도 있으면 메시지 전체를 버린다.
# 바퀴 토크가 꺼져 있으면(lekiwi/motor/wheel_torque로 켜지 않았으면) 쓰지 않고 버린다.
# 명령이 wheel_command_timeout(기본 0.5 s) 넘게 끊기면 드라이버가 속도 0을 쓴다.

std_msgs/Header header

uint8[] ids              # 모터 ID. velocity와 길이가 같아야 한다
float64[] velocity       # [rad/s] 모터 축 기준. 바퀴 방향(wheel_directions)은 보내는 쪽이 이미 적용했다
