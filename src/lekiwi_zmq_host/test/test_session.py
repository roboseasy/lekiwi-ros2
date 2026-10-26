"""session 시험. ROS 없이 돈다 (시각을 직접 준다)."""

import pytest

from lekiwi_zmq_host.protocol import Command
from lekiwi_zmq_host.session import (
	CREATE_CMD_VEL,
	DESTROY_CMD_VEL,
	DISABLE_WHEELS,
	ENABLE_ARM,
	ENABLE_WHEELS,
	EXTERNAL_TORQUE_OFF,
	PUBLISH_ARM,
	PUBLISH_BASE,
	PUBLISH_ZERO,
	SESSION_ACTIVE,
	SESSION_BLOCKED,
	SESSION_IDLE,
	STATUS_CHANGED,
	Session,
)

ARM = {'arm_gripper': 50.0}
BASE = (0.1, 0.0, 0.0)


def full(arm=True) -> Command:
	return Command(dict(ARM) if arm else None, BASE)


def kinds(effects) -> list[str]:
	return [effect.kind for effect in effects]


def started(now=0.0, arm=True) -> Session:
	session = Session()
	session.on_command(now, full(arm), [])
	assert session.state == SESSION_ACTIVE
	return session


def test_first_command_starts_the_session_in_order():
	session = Session()
	effects = session.on_command(0.0, full(), [])
	assert kinds(effects) == [
		ENABLE_WHEELS, CREATE_CMD_VEL, ENABLE_ARM, PUBLISH_ARM, PUBLISH_BASE, STATUS_CHANGED]
	assert effects[3].value == ARM and effects[4].value == BASE
	assert session.status() == {'session': SESSION_ACTIVE, 'reason': None}
	assert kinds(session.on_command(0.03, full(), [])) == [PUBLISH_ARM, PUBLISH_BASE]


def test_arm_torque_only_with_the_first_arm_command():
	session = Session()
	assert ENABLE_ARM not in kinds(session.on_command(0.0, full(arm=False), []))
	assert kinds(session.on_command(0.1, full(), [])) == [ENABLE_ARM, PUBLISH_ARM, PUBLISH_BASE]
	assert ENABLE_ARM not in kinds(session.on_command(0.2, full(), []))


@pytest.mark.parametrize('reason', [
	'웹 화면이 조작 중입니다', '자율 주행 중입니다 (collision_monitor 가 cmd_vel 을 보냅니다)',
	'다른 프로그램이 cmd_vel 을 보내고 있습니다: /teleop', '비상 정지가 걸려 있습니다',
	'팔로워 팔 노드(lekiwi_follower_arm_node)가 없습니다', '베이스 컨트롤러 파라미터를 읽지 못했습니다',
])
def test_each_start_blocker_refuses_without_touching_the_robot(reason):
	session = Session()
	assert kinds(session.on_command(0.0, full(), [reason, 'later'])) == [STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': reason}


def test_zero_once_after_half_a_second():
	session = started()
	assert session.on_tick(0.49) == []
	assert kinds(session.on_tick(0.5)) == [PUBLISH_ZERO]
	assert session.on_tick(0.6) == []
	session.on_command(0.7, full(), [])
	assert kinds(session.on_tick(1.2)) == [PUBLISH_ZERO]   # 다시 끊기면 다시 한 번


def test_session_ends_after_two_seconds_in_order():
	session = started()
	session.on_tick(0.5)
	assert kinds(session.on_tick(2.0)) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_IDLE, 'reason': None}
	assert session.on_tick(3.0) == []


def test_dropped_commands_do_not_keep_the_session_alive():
	session = started()
	session.on_command(1.9, Command(None, None, ['값이 유한한 수가 아닌 명령을 버립니다: x.vel']), [])
	assert DISABLE_WHEELS in kinds(session.on_tick(2.0))


def test_partial_arm_problem_is_the_reason_while_active():
	session = started()
	effects = session.on_command(0.1, Command(None, BASE, ['팔 관절 6개가 …']), [])
	assert kinds(effects) == [PUBLISH_BASE, STATUS_CHANGED]
	assert session.reason == '팔 관절 6개가 …'
	session.on_command(0.2, full(), [])
	assert session.reason is None


def test_external_wheel_torque_off_blocks_and_drops_cmd_vel_at_once():
	session = started()
	assert session.on_external(0.1, False, None) == []   # 켜는 중 (power true 직후 false)
	assert session.on_external(0.2, True, None) == []
	effects = session.on_external(0.3, False, None)
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': EXTERNAL_TORQUE_OFF}


def test_stale_wheel_ready_from_before_the_session_is_not_trusted():
	session = Session()
	session.on_external(-1.0, True, None)   # 웹이 켜 둔 바퀴
	session.on_command(0.0, full(), [])
	assert session.on_external(0.05, None, False) == []   # 팔 메시지
	assert session.on_external(0.1, False, None) == []    # power true 가 낸 false
	assert session.state == SESSION_ACTIVE


def test_external_arm_torque_off_after_arm_enable():
	session = started(arm=False)
	assert session.on_external(0.1, None, False) == []   # 팔 토크를 켜기 전
	session.on_command(0.2, full(), [])
	assert session.on_external(0.3, None, True) == []
	assert DESTROY_CMD_VEL in kinds(session.on_external(0.4, None, False))
	assert session.state == SESSION_BLOCKED


def test_arm_already_on_is_watched_from_its_current_value():
	session = Session()
	session.on_external(-1.0, None, True)   # 드라이버는 이미 켜진 팔에 arm_ready 를 다시 내지 않는다
	session.on_command(0.0, full(), [])
	assert DESTROY_CMD_VEL in kinds(session.on_external(0.1, None, False))


def test_blocked_keeps_refusing_until_two_seconds_of_silence_then_restarts():
	session = started()
	session.on_external(0.1, True, None)
	session.on_external(0.2, False, None)
	for step in range(1, 40):   # 계속 보내는 클라이언트: 토크를 다시 켜지 않는다
		now = 0.2 + step * 0.1
		assert session.on_command(now, full(), []) == []
		assert session.on_tick(now) == []
	assert session.state == SESSION_BLOCKED
	assert session.on_tick(now + 1.99) == []
	assert kinds(session.on_tick(now + 2.0)) == [STATUS_CHANGED]
	assert session.status() == {'session': SESSION_IDLE, 'reason': None}
	effects = session.on_command(now + 2.1, full(), [])
	assert kinds(effects)[:3] == [ENABLE_WHEELS, CREATE_CMD_VEL, ENABLE_ARM]
	assert session.state == SESSION_ACTIVE


def test_blocked_by_refusal_also_needs_silence():
	session = Session()
	session.on_command(0.0, full(), ['웹 화면이 조작 중입니다'])
	assert session.on_command(1.0, full(), []) == []   # 웹이 사라져도 끊기기 전에는 시작하지 않는다
	assert session.state == SESSION_BLOCKED
	session.on_tick(3.0)
	assert session.state == SESSION_IDLE


def test_dropped_command_does_not_overwrite_the_blocked_reason():
	session = Session()
	session.on_command(0.0, full(), ['비상 정지가 걸려 있습니다'])
	assert session.on_command(0.1, Command(None, None, ['명령을 읽을 수 없습니다']), []) == []
	assert session.reason == '비상 정지가 걸려 있습니다'


def test_service_failure_blocks_and_turns_our_wheels_off():
	session = started()
	effects = session.on_failure(0.1, '팔 토크 실패: ID 3')
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': '팔 토크 실패: ID 3'}
	assert Session().on_failure(0.0, 'x') == []
