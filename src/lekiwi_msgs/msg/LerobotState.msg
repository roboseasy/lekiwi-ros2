# LeRobot LeKiwi 관측 상태 한 샘플 (lekiwi_lerobot follower_arm → PC 데이터셋 기록기)
#
# lerobot LeKiwi(robot_type lekiwi_client) 데이터셋의 observation.state와 같은 값 · 순서 · 단위다.
# names는 늘 아래 9개, 이 순서다 (lerobot LeKiwi._state_ft):
#   arm_shoulder_pan.pos, arm_shoulder_lift.pos, arm_elbow_flex.pos, arm_wrist_flex.pos,
#   arm_wrist_roll.pos, arm_gripper.pos, x.vel, y.vel, theta.vel
# 팔 *.pos: lerobot 보정 단위 (몸 관절 [deg], 그리퍼 0~100, lerobot _normalize).
# x.vel · y.vel [m/s], theta.vel [deg/s]: 바퀴 Present_Velocity로 lerobot _wheel_raw_to_body를 계산한 값
# (바퀴 반지름 0.05 m, 몸체 반지름 0.125 m). ROS odom(몸체 반지름 0.13647 m, rad/s, 0.3 s 창)과 다르다.
# header.stamp는 이 값을 읽은 lekiwi/motor/states의 stamp다.
# 팔 6개와 바퀴 3개가 그 샘플에서 모두 응답했고, 모터 ID가 lerobot LeKiwi(팔 1~6, 바퀴 7~9)와 같고,
# 팔 보정이 드라이버가 읽은 레지스터(lekiwi/motor/arm_registers)와 맞을 때만 발행한다.

std_msgs/Header header

string[] names     # lerobot 특징 이름 (위 9개)
float64[] values   # names와 같은 순서 · 길이. 데이터셋에 쓸 때 float32로 바꾼다
