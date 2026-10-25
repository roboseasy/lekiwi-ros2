# 팔 모터 목표 위치 명령 (lekiwi_lerobot follower_arm → lekiwi_motor_driver)
#
# 드라이버는 이 메시지를 받으면 바로 SYNC_WRITE 한 번으로 이 메시지의 모든 ID를 Goal_Position(42)에 쓴다.
# 목표는 0~4095로 자르고, 그 모터의 마지막 Present_Position ± arm_max_goal_offset_ticks(기본 1024) 안으로 자른다.
# 위치를 읽지 못한 모터(이번 주기 online=false)가 하나라도 있으면 메시지 전체를 버린다.
# 드라이버 설정의 arm_ids에 없는 ID(예: 바퀴 7~9)가 하나라도 있으면 메시지 전체를 버린다.
# lekiwi/motor/arm_ready가 false면(lekiwi/motor/arm_torque로 켜지 않았으면) 쓰지 않고 버린다.
# 명령이 끊겨도 드라이버는 아무것도 하지 않는다. 모터는 마지막 목표를 유지한다 (팔이 떨어지지 않게).

std_msgs/Header header

uint8[] ids              # 모터 ID. positions_ticks와 길이가 같아야 한다
int32[] positions_ticks  # 0~4095 (한 바퀴 4096). 범위 밖 값은 드라이버가 자른다. 단위 · 보정 변환은 보내는 쪽이 했다
