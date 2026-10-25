# 팔 모터 EEPROM 설정값 (lekiwi_motor_driver → lekiwi_lerobot follower_arm)
#
# 드라이버가 arm_write_enabled일 때 arm_ids 모터에서 읽는다 (읽기만 한다. EEPROM에는 쓰지 않는다).
# 시작할 때 한 번, 그 뒤 처음 응답하지 않던 모터를 다시 읽을 때마다 발행한다 (reliable + transient_local, 깊이 1).
# 모든 배열은 ids와 길이 · 순서가 같다. valid가 false인 모터의 나머지 값은 0이다 (아직 읽지 못함).
# 단위는 모두 raw 레지스터 값이다. LeRobot 보정 JSON(calibration/robots/...)과 1:1로 비교할 수 있다.

std_msgs/Header header

uint8[] ids                  # 모터 ID (드라이버 arm_ids 순서)
bool[] valid                 # 이 모터를 읽었다
int32[] homing_offset        # Homing_Offset(31) [tick]. 부호-크기 bit 11을 푼 값 (LeRobot homing_offset과 같은 값)
int32[] min_position_limit   # Min_Position_Limit(9) [tick] (LeRobot range_min)
int32[] max_position_limit   # Max_Position_Limit(11) [tick] (LeRobot range_max)
uint8[] p_coefficient        # P_Coefficient(21)
uint8[] d_coefficient        # D_Coefficient(22)
uint8[] i_coefficient        # I_Coefficient(23)
