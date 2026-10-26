"""session 시험. ROS 없이 돈다 (시각을 직접 준다)."""

import pytest

from lekiwi_zmq_host.protocol import Command
from lekiwi_zmq_host.session import (
	CREATE_CMD_VEL,
	DESTROY_CMD_VEL,
	DISABLE_WHEELS,
	ENABLE_ARM,
	ENABLE_WHEELS,
	ESTOP_LATCHED,
	EXTERNAL_TORQUE_OFF,
	PUBLISH_ARM,
	PUBLISH_BASE,
	PUBLISH_ZERO,
	SESSION_ACTIVE,
	SESSION_BLOCKED,
	SESSION_IDLE,
	SESSION_WAITING,
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
	session.on_command(now, full(arm), lambda: [])
	assert session.state == SESSION_ACTIVE
	return session


def test_first_command_starts_the_session_in_order():
	session = Session()
	effects = session.on_command(0.0, full(), lambda: [])
	assert kinds(effects) == [
		ENABLE_WHEELS, CREATE_CMD_VEL, ENABLE_ARM, PUBLISH_ARM, PUBLISH_BASE, STATUS_CHANGED]
	assert effects[3].value == ARM and effects[4].value == BASE
	assert session.status() == {'session': SESSION_ACTIVE, 'reason': None, 'rejected_peer': None}
	assert kinds(session.on_command(0.03, full(), lambda: [])) == [PUBLISH_ARM, PUBLISH_BASE]


def test_arm_torque_only_with_the_first_arm_command():
	session = Session()
	assert ENABLE_ARM not in kinds(session.on_command(0.0, full(arm=False), lambda: []))
	assert kinds(session.on_command(0.1, full(), lambda: [])) == [ENABLE_ARM, PUBLISH_ARM, PUBLISH_BASE]
	assert ENABLE_ARM not in kinds(session.on_command(0.2, full(), lambda: []))


@pytest.mark.parametrize('reason', [
	'웹 화면이 조작 중입니다', '자율 주행 중입니다 (collision_monitor 가 cmd_vel 을 보냅니다)',
	'다른 프로그램이 cmd_vel 을 보내고 있습니다: /teleop', '비상 정지가 걸려 있습니다',
	'팔로워 팔 노드(lekiwi_follower_arm_node)가 없습니다', '베이스 컨트롤러 파라미터를 읽지 못했습니다',
])
def test_each_start_blocker_waits_without_touching_the_robot(reason):
	session = Session()
	assert kinds(session.on_command(0.0, full(), lambda: [reason, 'later'])) == [STATUS_CHANGED]
	assert session.status() == {'session': SESSION_WAITING, 'reason': reason, 'rejected_peer': None}


def test_zero_once_after_half_a_second():
	session = started()
	assert session.on_tick(0.49) == []
	assert kinds(session.on_tick(0.5)) == [PUBLISH_ZERO]
	assert session.on_tick(0.6) == []
	session.on_command(0.7, full(), lambda: [])
	assert kinds(session.on_tick(1.2)) == [PUBLISH_ZERO]   # 다시 끊기면 다시 한 번


def test_session_ends_after_two_seconds_in_order():
	session = started()
	session.on_tick(0.5)
	assert kinds(session.on_tick(2.0)) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_IDLE, 'reason': None, 'rejected_peer': None}
	assert session.on_tick(3.0) == []


def test_dropped_commands_do_not_keep_the_session_alive():
	session = started()
	session.on_command(1.9, Command(None, None, ['값이 유한한 수가 아닌 명령을 버립니다: x.vel']), lambda: [])
	assert DISABLE_WHEELS in kinds(session.on_tick(2.0))


def test_partial_arm_problem_is_the_reason_while_active():
	session = started()
	effects = session.on_command(0.1, Command(None, BASE, ['팔 관절 6개가 …']), lambda: [])
	assert kinds(effects) == [PUBLISH_BASE, STATUS_CHANGED]
	assert session.reason == '팔 관절 6개가 …'
	session.on_command(0.2, full(), lambda: [])
	assert session.reason is None


def test_external_wheel_torque_off_blocks_and_drops_cmd_vel_at_once():
	session = started()
	assert session.on_external(0.1, False, None) == []   # 켜는 중 (power true 직후 false)
	assert session.on_external(0.2, True, None) == []
	effects = session.on_external(0.3, False, None)
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': EXTERNAL_TORQUE_OFF, 'rejected_peer': None}


def test_stale_wheel_ready_from_before_the_session_is_not_trusted():
	session = Session()
	session.on_external(-1.0, True, None)   # 웹이 켜 둔 바퀴
	session.on_command(0.0, full(), lambda: [])
	assert session.on_external(0.05, None, False) == []   # 팔 메시지
	assert session.on_external(0.1, False, None) == []    # power true 가 낸 false
	assert session.state == SESSION_ACTIVE


def test_external_arm_torque_off_after_arm_enable():
	session = started(arm=False)
	assert session.on_external(0.1, None, False) == []   # 팔 토크를 켜기 전
	session.on_command(0.2, full(), lambda: [])
	assert session.on_external(0.3, None, True) == []
	assert DESTROY_CMD_VEL in kinds(session.on_external(0.4, None, False))
	assert session.state == SESSION_BLOCKED


def test_arm_already_on_is_watched_from_its_current_value():
	session = Session()
	session.on_external(-1.0, None, True)   # 드라이버는 이미 켜진 팔에 arm_ready 를 다시 내지 않는다
	session.on_command(0.0, full(), lambda: [])
	assert DESTROY_CMD_VEL in kinds(session.on_external(0.1, None, False))


def test_blocked_keeps_refusing_until_two_seconds_of_silence_then_restarts():
	session = started()
	session.on_external(0.1, True, None)
	session.on_external(0.2, False, None)
	for step in range(1, 40):   # 계속 보내는 클라이언트: 토크를 다시 켜지 않는다
		now = 0.2 + step * 0.1
		assert session.on_command(now, full(), lambda: []) == []
		assert session.on_tick(now) == []
	assert session.state == SESSION_BLOCKED
	assert session.on_tick(now + 1.99) == []
	assert kinds(session.on_tick(now + 2.0)) == [DESTROY_CMD_VEL, STATUS_CHANGED]   # 남은 발행자 대비
	assert session.status() == {'session': SESSION_IDLE, 'reason': None, 'rejected_peer': None}
	effects = session.on_command(now + 2.1, full(), lambda: [])
	assert kinds(effects)[:3] == [ENABLE_WHEELS, CREATE_CMD_VEL, ENABLE_ARM]
	assert session.state == SESSION_ACTIVE


def test_waiting_rechecks_once_a_second_then_starts_in_order():
	session = Session()
	checks = []

	def blockers():
		checks.append(True)
		return ['웹 화면이 조작 중입니다'] if len(checks) < 3 else []

	session.on_command(0.0, full(), blockers)
	for step in range(1, 33):   # 30 Hz 로 계속 보내는 클라이언트: 1 s 가 지나야 다시 본다
		assert session.on_command(step / 33.0, full(), blockers) == []
		assert session.on_tick(step / 33.0) == []
	assert len(checks) == 1 and session.state == SESSION_WAITING
	assert session.on_command(1.0, full(), blockers) == []   # 다시 봤지만 아직 막힘
	assert len(checks) == 2 and session.state == SESSION_WAITING
	assert session.on_command(1.5, full(), blockers) == [] and len(checks) == 2
	effects = session.on_command(2.0, full(), blockers)
	assert kinds(effects) == [
		ENABLE_WHEELS, CREATE_CMD_VEL, ENABLE_ARM, PUBLISH_ARM, PUBLISH_BASE, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_ACTIVE, 'reason': None, 'rejected_peer': None}


def test_waiting_drops_commands_and_ends_after_two_seconds_of_silence():
	session = Session()
	session.on_command(0.0, full(), lambda: ['팔로워 팔 노드(lekiwi_follower_arm_node)가 없습니다'])
	assert session.on_command(0.5, full(), lambda: []) == []   # 1 s 전: 다시 보지 않고 버린다
	assert session.on_tick(1.0) == []   # 0 도 보내지 않는다 (발행자가 없다)
	assert session.on_tick(2.49) == []
	assert kinds(session.on_tick(2.5)) == [STATUS_CHANGED]   # 토크 · 발행자를 건드리지 않는다
	assert session.status() == {'session': SESSION_IDLE, 'reason': None, 'rejected_peer': None}


def test_dropped_command_does_not_overwrite_the_waiting_reason():
	session = Session()
	session.on_command(0.0, full(), lambda: ['비상 정지가 걸려 있습니다'])
	assert session.on_command(0.1, Command(None, None, ['명령을 읽을 수 없습니다']), lambda: []) == []
	assert session.reason == '비상 정지가 걸려 있습니다'


def test_service_failure_blocks_and_turns_our_wheels_off():
	session = started()
	effects = session.on_failure(0.1, '팔 토크 실패: ID 3')
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': '팔 토크 실패: ID 3', 'rejected_peer': None}
	assert Session().on_failure(0.0, 'x') == []


def test_external_arm_torque_off_also_turns_our_wheels_off():
	session = started()
	session.on_external(0.1, True, True)
	effects = session.on_external(0.2, None, False)   # 게이트웨이가 팔 힘을 뺐다 · 바퀴는 켜진 채
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.on_external(0.3, False, None) == []   # 다시 끄지 않는다
	assert kinds(session.on_tick(2.2)) == [DESTROY_CMD_VEL, STATUS_CHANGED]


def test_blocked_from_active_still_needs_two_seconds_of_silence():
	session = started()
	session.on_failure(0.1, '바퀴 토크 실패')
	assert session.on_command(1.2, full(), lambda: []) == []   # 1 s 다시 보기는 waiting 에만
	assert session.state == SESSION_BLOCKED
	assert session.on_tick(3.19) == []
	assert DESTROY_CMD_VEL in kinds(session.on_tick(3.2))
	assert session.state == SESSION_IDLE


def test_refused_start_never_turns_wheels_off():
	session = Session()
	session.on_command(0.0, full(), lambda: ['비상 정지가 걸려 있습니다'])
	assert DISABLE_WHEELS not in kinds(session.on_tick(2.0))


def test_estop_while_active_blocks_even_before_wheels_are_ready():
	session = started(arm=False)
	assert session.on_external(0.1, False, None) == []   # 엔코더 기다리는 중
	effects = session.on_estop(0.2, True)
	assert kinds(effects) == [PUBLISH_ZERO, DESTROY_CMD_VEL, DISABLE_WHEELS, STATUS_CHANGED]
	assert session.status() == {'session': SESSION_BLOCKED, 'reason': ESTOP_LATCHED, 'rejected_peer': None}
	assert session.on_command(0.3, full(), lambda: []) == []
	assert session.on_estop(0.4, True) == []
	assert Session().on_estop(0.0, True) == []   # idle 은 시작 거절이 맡는다
	assert started().on_estop(0.0, False) == []


def test_commands_from_another_peer_are_dropped_and_do_not_feed_the_watchdog():
	session = Session()
	session.on_command(0.0, full(), lambda: [], peer='10.0.0.2')
	assert session.owner == '10.0.0.2'
	assert kinds(session.on_command(0.1, full(), lambda: [], peer='10.0.0.3')) == [STATUS_CHANGED]
	assert session.status() == {
		'session': SESSION_ACTIVE, 'reason': '다른 PC(10.0.0.2)가 조작 중입니다', 'rejected_peer': '10.0.0.3'}
	assert session.on_command(0.2, full(), lambda: [], peer='10.0.0.3') == []   # 같은 이유: 상태는 그대로
	for now in (1.0, 2.0, 3.0):
		assert kinds(session.on_command(now, full(), lambda: [], peer='10.0.0.2')) == [PUBLISH_ARM, PUBLISH_BASE]
	assert session.on_tick(3.1) == []
	assert kinds(session.on_tick(3.2)) == [STATUS_CHANGED]   # 마지막 거절 3 s 뒤 지운다
	assert session.status()['rejected_peer'] is None
	for step in range(1, 17):
		session.on_command(3.0 + step * 0.1, full(), lambda: [], peer='10.0.0.3')
	assert DISABLE_WHEELS in kinds(session.on_tick(5.0))   # 주인 마지막 명령 3.0 에서 2 s: 다른 PC 가 늘리지 않는다
	assert session.owner is None and session.status()['rejected_peer'] is None
	session.on_command(5.1, full(), lambda: [], peer='10.0.0.3')   # idle 뒤에는 새 주인
	assert session.state == SESSION_ACTIVE and session.owner == '10.0.0.3'


def test_another_peer_cannot_keep_a_blocked_or_waiting_session_alive():
	for start in (lambda s: s.on_command(0.0, full(), lambda: ['웹 화면이 조작 중입니다'], peer='a'),
			lambda s: (s.on_command(0.0, full(), lambda: [], peer='a'), s.on_failure(0.1, 'x'))):
		session = Session()
		start(session)
		checks = []
		for step in range(1, 30):
			session.on_command(step * 0.1, full(), lambda: checks.append(1) or [], peer='b')
		assert checks == [] and session.state in (SESSION_WAITING, SESSION_BLOCKED)
		assert session.on_tick(2.0)[-1].kind == STATUS_CHANGED and session.state == SESSION_IDLE


def test_unknown_peer_skips_the_owner_check():
	session = Session()
	session.on_command(0.0, full(), lambda: [])
	assert kinds(session.on_command(0.1, full(), lambda: [], peer='10.0.0.3')) == [PUBLISH_ARM, PUBLISH_BASE]
