"""follower_logic 시험. ROS 없이 돈다.

ArmConverter 시험은 lerobot이 있을 때만 돈다 (없으면 건너뛴다):
	/home/<user>/miniconda3/envs/lerobot060/bin/python -m pytest test
FollowerController · FollowerArmLogic · base_velocity 시험은 시스템 python(colcon test)에서도 돈다
(변환은 lerobot 공식을 그대로 쓴 FakeConverter). base_velocity를 lerobot 함수와 맞춰 보는 시험은 lerobot이 있을 때만.
"""

import array
import json
import math
import random
from collections.abc import Callable
from pathlib import Path
from types import SimpleNamespace

import pytest

from lekiwi_lerobot.follower_logic import (
	ARM_IDS,
	ARM_JOINTS,
	BASE_WHEELS,
	GRIPPER,
	LEROBOT_STATE_NAMES,
	STATE_ERROR,
	STATE_FOLLOWING,
	STATE_IDLE,
	STATE_RAMPING,
	STATE_RESTED,
	STATE_RESTING,
	ArmConverter,
	FollowerArmLogic,
	FollowerController,
	arm_motor_error,
	base_velocity,
	compare_registers,
	lerobot_version_error,
	limit_ticks,
	make_status,
	wheel_raw_to_body,
)

# lekiwi04 보정 사본 (2026-10-22 로봇에서 가져온 것)
CALIBRATION = Path(__file__).resolve().parent / 'data' / 'lekiwi04.json'
MAX_TICK = 4095   # lerobot DEGREES 계산의 분모 (해상도 4096 - 1)
PERIOD = 1.0 / 30.0
BODY_JOINTS = tuple(name for name in ARM_JOINTS if name != GRIPPER)


def calibration_ranges() -> dict[str, tuple[int, int]]:
	"""보정 파일의 팔 (range_min, range_max)."""
	data = json.loads(CALIBRATION.read_text(encoding='utf-8'))
	return {
		name: (data[name]['range_min'], data[name]['range_max'])
		for name in ARM_JOINTS}


def plain_units(name: str, tick: float, ranges: dict) -> float:
	"""lerobot 계산을 쓰지 않은 공식 (몸 관절 각도, 그리퍼 0~100)."""
	range_min, range_max = ranges[name]
	if name == GRIPPER:
		bounded = min(range_max, max(range_min, tick))
		return (bounded - range_min) / (range_max - range_min) * 100.0
	return (tick - (range_min + range_max) / 2.0) * 360.0 / MAX_TICK


# ── ArmConverter (lerobot 필요) ─────────────────────────
@pytest.fixture(scope='module')
def converter() -> ArmConverter:
	pytest.importorskip('lerobot')
	return ArmConverter(CALIBRATION)


def test_to_units_matches_plain_formula(converter: ArmConverter) -> None:
	ranges = calibration_ranges()
	for name in ARM_JOINTS:
		range_min, range_max = ranges[name]
		# 범위 밖 (접은 자세 팔꿈치 3190 > 3094) · 모터 끝까지 본다
		for tick in (0, range_min, (range_min + range_max) // 2, range_max,
				3190, MAX_TICK):
			assert converter.to_units({name: tick})[name] == pytest.approx(
				plain_units(name, tick, ranges), abs=1e-9), (name, tick)


def test_round_trip_is_exact_over_whole_range(converter: ArmConverter) -> None:
	for name, (range_min, range_max) in calibration_ranges().items():
		for tick in range(range_min, range_max + 1):
			units = converter.to_units({name: tick})
			assert converter.to_ticks(units)[name] == tick, (name, tick)


def test_limits_and_units_per_degree(converter: ArmConverter) -> None:
	ranges = calibration_ranges()
	assert converter.limits['arm_shoulder_pan'] == pytest.approx(
		(-120.615, 120.615), abs=1e-3)
	assert converter.limits['arm_elbow_flex'] == pytest.approx(
		(-97.231, 97.231), abs=1e-3)
	assert converter.limits['arm_wrist_roll'] == pytest.approx(
		(-180.0, 180.0), abs=1e-9)
	assert converter.limits[GRIPPER] == (0.0, 100.0)
	for name in BODY_JOINTS:
		assert converter.units_per_degree[name] == pytest.approx(1.0)
	range_min, range_max = ranges[GRIPPER]
	assert converter.units_per_degree[GRIPPER] == pytest.approx(
		100.0 / ((range_max - range_min) * 360.0 / MAX_TICK))
	assert converter.ids == {name: i + 1 for i, name in enumerate(ARM_JOINTS)}
	data = json.loads(CALIBRATION.read_text(encoding='utf-8'))
	assert converter.homing_offsets == {
		name: data[name]['homing_offset'] for name in ARM_JOINTS}


def test_to_ticks_clips_to_motor_and_limits_give_range(
	converter: ArmConverter,
) -> None:
	# lerobot만으로는 999° → 13351 tick
	assert converter.to_ticks({'arm_shoulder_pan': 999.0}) == {
		'arm_shoulder_pan': MAX_TICK}
	assert converter.to_ticks({'arm_shoulder_pan': -999.0}) == {
		'arm_shoulder_pan': 0}
	# 목표를 limits로 자르면 (FollowerController) 정확히 보정 범위 끝이 된다
	for name, (range_min, range_max) in calibration_ranges().items():
		low, high = converter.limits[name]
		assert converter.to_ticks({name: low})[name] == range_min
		assert converter.to_ticks({name: high})[name] == range_max
	# 그리퍼는 lerobot이 0~100으로 자른다
	assert converter.to_ticks({GRIPPER: 150.0})[GRIPPER] == (
		calibration_ranges()[GRIPPER][1])


def test_bad_calibration_raises(tmp_path: Path) -> None:
	pytest.importorskip('lerobot')
	data = json.loads(CALIBRATION.read_text(encoding='utf-8'))
	del data[GRIPPER]
	missing = tmp_path / 'missing.json'
	missing.write_text(json.dumps(data), encoding='utf-8')
	with pytest.raises(ValueError, match='arm_gripper'):
		ArmConverter(missing)

	data = json.loads(CALIBRATION.read_text(encoding='utf-8'))
	data['arm_elbow_flex']['id'] = 9
	wrong_id = tmp_path / 'wrong_id.json'
	wrong_id.write_text(json.dumps(data), encoding='utf-8')
	with pytest.raises(ValueError, match='arm_elbow_flex'):
		ArmConverter(wrong_id)

	with pytest.raises(OSError):
		ArmConverter(tmp_path / 'none.json')


def test_other_lerobot_version_raises(monkeypatch: pytest.MonkeyPatch) -> None:
	lerobot = pytest.importorskip('lerobot')
	assert lerobot_version_error(lerobot.__version__) is None   # 시험하는 이 환경
	monkeypatch.setattr(lerobot, '__version__', '0.7.0')
	with pytest.raises(RuntimeError, match='0.7.0'):
		ArmConverter(CALIBRATION)


# ── FollowerController (lerobot 없이) ───────────────────
# lekiwi04 보정 범위 · 그리퍼 0~100 단위 (ArmConverter 값을 반올림)
LIMITS = {
	'arm_shoulder_pan': (-120.6, 120.6),
	'arm_shoulder_lift': (-106.1, 106.1),
	'arm_elbow_flex': (-97.2, 97.2),
	'arm_wrist_flex': (-100.8, 100.8),
	'arm_wrist_roll': (-180.0, 180.0),
	GRIPPER: (0.0, 100.0),
}
GRIPPER_SCALE = 0.7   # lekiwi04 약 0.704 단위/°
SCALES = {name: 1.0 for name in BODY_JOINTS} | {GRIPPER: GRIPPER_SCALE}
ZERO = {name: 0.0 for name in ARM_JOINTS}
STEP = 10.0   # max_step_deg 기본값
Pose = dict[str, float]


def make_controller(**settings: float) -> FollowerController:
	return FollowerController(LIMITS, SCALES, **settings)


def pose(**values: float) -> Pose:
	"""0 자세에서 일부 관절만 바꾼 자세."""
	return ZERO | values


def update(
	controller: FollowerController,
	now: float,
	follower: Pose,
	goal: Pose | None,
	goal_time: float,
) -> Pose | None:
	"""드라이버가 팔 명령을 받는 (arm_ready) 한 주기."""
	return controller.update(now, follower, goal, goal_time, True)


# 가짜 모터: (잰 자세, 이번 명령) → 다음 주기에 잴 자세
def ideal_motor(measured: Pose, command: Pose | None) -> Pose:
	return measured if command is None else dict(command)


def frozen_motor(measured: Pose, command: Pose | None) -> Pose:
	"""팔이 막혀 움직이지 않는다."""
	return measured


def slow_motor(degrees_per_cycle: float):
	"""명령을 받지만 한 주기에 degrees_per_cycle만 간다 (무거운 팔 · 빠른 리더)."""
	def motor(measured: Pose, command: Pose | None) -> Pose:
		if command is None:
			return measured
		return {
			name: value + min(degrees_per_cycle, max(
				-degrees_per_cycle, command[name] - value))
			for name, value in measured.items()}
	return motor


def gripper_blocked_at(value: float):
	"""그리퍼가 물건에 막혀 value에 멈추고 나머지는 명령대로 간다."""
	def motor(measured: Pose, command: Pose | None) -> Pose:
		return ideal_motor(measured, command) | {GRIPPER: value}
	return motor


def run(
	controller: FollowerController,
	follower: Pose,
	goal: Pose,
	start: float,
	cycles: int,
	motor=ideal_motor,
) -> list[tuple[float, Pose, Pose | None, str]]:
	"""30 Hz로 목표를 새로 받으며 돈다: (시각, 잰 자세, 명령, 상태) 목록.

	명령이 None이 되면 (그쳤다) 그 주기에서 멈춘다.
	"""
	steps = []
	measured = dict(follower)
	for cycle in range(cycles):
		now = start + cycle * PERIOD
		command = update(controller, now, measured, goal, now)
		steps.append((now, measured, command, controller.state))
		if command is None:
			break
		measured = motor(measured, command)
	return steps


def commands(steps: list, name: str) -> list[float]:
	return [command[name] for _, _, command, _ in steps if command is not None]


def test_no_goal_is_idle_and_sends_nothing() -> None:
	controller = make_controller()
	assert update(controller, 0.0, ZERO, None, 0.0) is None
	assert controller.state == STATE_IDLE
	assert controller.reason
	assert controller.diffs == {}


def test_small_difference_follows_without_ramp() -> None:
	controller = make_controller()
	command = update(controller, 0.0, ZERO, pose(arm_shoulder_pan=5.0), 0.0)
	assert controller.state == STATE_FOLLOWING
	assert controller.ramp_progress is None
	assert command == pytest.approx(pose(arm_shoulder_pan=5.0))
	assert controller.diffs['arm_shoulder_pan'] == pytest.approx(5.0)


def test_ramp_threshold_is_strictly_greater() -> None:
	controller = make_controller()
	update(controller, 0.0, ZERO, pose(arm_elbow_flex=-8.0), 0.0)
	assert controller.state == STATE_FOLLOWING
	controller = make_controller()
	update(controller, 0.0, ZERO, pose(arm_elbow_flex=-8.1), 0.0)
	assert controller.state == STATE_RAMPING
	assert 'arm_elbow_flex' in controller.reason


def test_far_goal_ramps_over_ramp_time_with_smooth_shape() -> None:
	controller = make_controller()
	steps = run(controller, ZERO, pose(arm_shoulder_lift=60.0), 10.0, 70)
	lifts = commands(steps, 'arm_shoulder_lift')
	assert len(lifts) == 70

	def smoothstep(progress: float) -> float:
		return progress * progress * (3.0 - 2.0 * progress)

	for now, _, command, state in steps:
		progress = (now - 10.0) / 2.0
		assert command['arm_shoulder_lift'] == pytest.approx(
			60.0 * smoothstep(min(1.0, progress)))
		if progress < 0.999:
			assert state == STATE_RAMPING
		elif progress > 1.001:   # 2초 바로 그 주기는 부동소수 반올림에 맡긴다
			assert state == STATE_FOLLOWING
	# 처음 · 끝은 느리고 가운데가 빠르다 (smoothstep), 늘 한 방향
	assert lifts[0] == 0.0
	assert all(b >= a for a, b in zip(lifts, lifts[1:]))
	assert lifts[2] - lifts[1] < lifts[31] - lifts[30]
	assert lifts[59] - lifts[58] < lifts[31] - lifts[30]
	# 다른 관절은 그대로
	assert all(
		command[name] == 0.0 for _, _, command, _ in steps
		for name in ARM_JOINTS if name != 'arm_shoulder_lift')


def test_ramp_follows_a_moving_goal_and_reports_progress() -> None:
	controller = make_controller()
	update(controller, 0.0, ZERO, pose(arm_shoulder_pan=40.0), 0.0)
	assert controller.ramp_progress == 0.0
	command = update(controller, 1.0, ZERO, pose(arm_shoulder_pan=30.0), 1.0)
	assert controller.ramp_progress == pytest.approx(0.5)
	# smoothstep(0.5) = 0.5: 지금 목표 30의 절반 15. 잰 자세 0에서 10° 까지만
	assert command['arm_shoulder_pan'] == pytest.approx(STEP)
	measured = pose(arm_shoulder_pan=STEP)
	command = update(
		controller, 1.05, measured, pose(arm_shoulder_pan=30.0), 1.05)
	assert command['arm_shoulder_pan'] == pytest.approx(
		30.0 * 0.525 * 0.525 * (3.0 - 2.0 * 0.525))


FOLLOW_STEP = 45.0   # follow_step_deg 기본값


def test_following_sends_the_goal_and_clips_only_jumps_from_the_last_command() -> None:
	controller = make_controller()
	update(controller, 0.0, ZERO, ZERO, 0.0)
	assert controller.state == STATE_FOLLOWING
	# 사람 손 속도(한 주기 14°)는 그대로, 50° 튐은 지난 명령에서 45°까지만
	jump = pose(arm_wrist_roll=50.0, arm_wrist_flex=-14.0)
	steps = run(controller, ZERO, jump, PERIOD, 4)
	assert commands(steps, 'arm_wrist_roll') == pytest.approx([45, 50, 50, 50])
	assert commands(steps, 'arm_wrist_flex') == pytest.approx([-14] * 4)
	assert all(state == STATE_FOLLOWING for _, _, _, state in steps)
	assert controller.is_step_from_command
	# 손목 이음매 (+179 → -179): 반대쪽으로 한 번에 358° 가지 않고 45°씩
	controller = make_controller()
	update(controller, 0.0, pose(arm_wrist_roll=179.0), pose(arm_wrist_roll=179.0), 0.0)
	steps = run(controller, pose(arm_wrist_roll=179.0), pose(arm_wrist_roll=-179.0), PERIOD, 3)
	assert commands(steps, 'arm_wrist_roll') == pytest.approx([134, 89, 44])


def test_following_does_not_wait_for_the_measured_pose_but_the_first_command_does() -> None:
	# 따라가는 중: 팔이 늦어도 (여기서는 안 움직여도) 명령은 목표로 간다 (lerobot max_relative_target None)
	controller = make_controller()
	update(controller, 0.0, ZERO, ZERO, 0.0)
	steps = run(controller, ZERO, pose(arm_shoulder_pan=50.0), PERIOD, 4,
		motor=frozen_motor)
	assert commands(steps, 'arm_shoulder_pan') == pytest.approx([45.0, 50.0, 50.0, 50.0])
	# 멈춘 뒤 첫 명령은 잰 자세에서 10° 안이다 (차이가 램프 문턱 8°보다 작아도)
	controller = make_controller()
	command = update(controller, 0.0, ZERO, pose(arm_shoulder_pan=7.0), 0.0)
	assert controller.state == STATE_FOLLOWING and not controller.is_step_from_command
	assert command['arm_shoulder_pan'] == pytest.approx(7.0)
	command = update(
		controller, 0.0, pose(arm_shoulder_pan=40.0), pose(arm_shoulder_pan=47.0), 0.0)
	assert controller.is_step_from_command   # 그다음부터는 지난 명령 기준
	assert command['arm_shoulder_pan'] == pytest.approx(47.0)


def test_max_step_and_follow_step_settings_change_the_steps() -> None:
	controller = make_controller(max_step_deg=2.0, track_tolerance_deg=1.0, rest_tolerance_deg=1.0)
	command = update(controller, 0.0, ZERO, pose(arm_shoulder_pan=7.0), 0.0)   # 멈춘 뒤 첫 명령
	assert command['arm_shoulder_pan'] == pytest.approx(2.0)
	controller = make_controller(follow_step_deg=20.0)
	update(controller, 0.0, ZERO, ZERO, 0.0)
	command = update(controller, PERIOD, ZERO, pose(arm_shoulder_pan=50.0), PERIOD)
	assert command['arm_shoulder_pan'] == pytest.approx(20.0)


def test_gripper_uses_equivalent_threshold_and_step() -> None:
	# 8° = 5.6 단위: 5.5는 램프 없이, 5.7은 램프
	controller = make_controller()
	update(controller, 0.0, ZERO, pose(arm_gripper=5.5), 0.0)
	assert controller.state == STATE_FOLLOWING
	controller = make_controller()
	update(controller, 0.0, ZERO, pose(arm_gripper=5.7), 0.0)
	assert controller.state == STATE_RAMPING
	# 멈춘 뒤 첫 명령: 10° = 7.0 단위까지
	controller = make_controller(ramp_threshold_deg=20.0)
	command = update(controller, 0.0, ZERO, pose(arm_gripper=10.0), 0.0)
	assert controller.state == STATE_FOLLOWING
	assert command[GRIPPER] == pytest.approx(10 * GRIPPER_SCALE)
	# 따라가는 중: 45° = 31.5 단위씩
	controller = make_controller()
	update(controller, 0.0, ZERO, ZERO, 0.0)
	steps = run(controller, ZERO, pose(arm_gripper=80.0), PERIOD, 3)
	assert commands(steps, GRIPPER) == pytest.approx([31.5, 63.0, 80.0])


def test_goal_is_clipped_to_calibration_limits() -> None:
	controller = make_controller()
	update(controller, 0.0, ZERO, ZERO, 0.0)
	goal = pose(arm_shoulder_pan=999.0, arm_gripper=-20.0)
	steps = run(controller, ZERO, goal, PERIOD, 40)
	assert steps[-1][2]['arm_shoulder_pan'] == pytest.approx(120.6)
	assert steps[-1][2][GRIPPER] == 0.0
	assert max(commands(steps, 'arm_shoulder_pan')) <= 120.6
	update(controller, 41 * PERIOD, ZERO, goal, 41 * PERIOD)
	assert controller.diffs['arm_shoulder_pan'] == pytest.approx(120.6)


def test_out_of_range_start_comes_inside_without_jump() -> None:
	# lekiwi04 접은 자세: 팔꿈치 105.7° (보정 범위 97.2° 밖)
	follower = pose(arm_elbow_flex=105.7)
	controller = make_controller()
	steps = run(controller, follower, pose(arm_elbow_flex=90.0), 0.0, 70)
	elbows = commands(steps, 'arm_elbow_flex')
	assert steps[0][3] == STATE_RAMPING   # 자른 목표 90 과 15.7° 차이
	assert elbows[0] == pytest.approx(105.7)
	assert all(90.0 <= elbow <= 105.7 for elbow in elbows)
	assert all(b <= a for a, b in zip(elbows, elbows[1:]))
	assert elbows[-1] == pytest.approx(90.0)


def test_goal_timeout_holds_and_next_goal_ramps_again() -> None:
	controller = make_controller()
	goal = pose(arm_shoulder_pan=3.0)
	assert update(controller, 0.0, ZERO, goal, 0.0) is not None
	assert update(controller, 0.5, ZERO, goal, 0.0) is not None   # 0.5 s까지는 산다
	assert update(controller, 0.51, ZERO, goal, 0.0) is None
	assert controller.state == STATE_IDLE
	assert '유지' in controller.reason
	assert controller.diffs == {}
	assert update(controller, 5.0, ZERO, goal, 0.0) is None   # 계속 유지

	# 그동안 팔로워가 움직였다 (손으로 밀었다): 새 목표는 그 자세에서 램프
	moved = pose(arm_shoulder_pan=30.0)
	command = update(controller, 6.0, moved, goal, 6.0)
	assert controller.state == STATE_RAMPING
	assert command['arm_shoulder_pan'] == pytest.approx(30.0)
	steps = run(controller, moved, goal, 6.0 + PERIOD, 70)
	assert steps[-1][3] == STATE_FOLLOWING
	assert steps[-1][2]['arm_shoulder_pan'] == pytest.approx(3.0)

	# 가까운 자세로 다시 시작하면 램프 없이 따라간다
	update(controller, 20.0, ZERO, goal, 0.0)
	assert controller.state == STATE_IDLE
	update(controller, 21.0, pose(arm_shoulder_pan=1.0), goal, 21.0)
	assert controller.state == STATE_FOLLOWING


def test_frozen_arm_in_ramp_warns_and_keeps_commanding() -> None:
	# (b) 팔이 막혔다: 경고만 하고 명령은 잰 자세 + 10° 안에서 계속 낸다
	controller = make_controller()
	goal = pose(arm_shoulder_pan=60.0)
	warned_at = None
	measured = ZERO
	for cycle in range(60):
		now = cycle * PERIOD
		command = update(controller, now, measured, goal, now)
		assert command is not None
		assert command['arm_shoulder_pan'] <= STEP + 1e-9
		if warned_at is None and controller.warning is not None:
			warned_at = now
	assert controller.state in (STATE_RAMPING, STATE_FOLLOWING)
	assert 0.5 < warned_at < 0.8   # 4°를 넘은 뒤 0.3 s
	assert '따라오지' in controller.warning
	assert 'arm_shoulder_pan' in controller.warning
	assert controller.reason == '' or '맞춥니다' in controller.reason
	# 풀리면 따라가고 경고가 사라진다
	steps = run(controller, ZERO, goal, 2.0, 20)
	assert steps[-1][2]['arm_shoulder_pan'] == pytest.approx(60.0)
	assert controller.warning is None


def test_blocked_joint_while_following_warns_without_halt() -> None:
	controller = make_controller()
	run(controller, ZERO, pose(arm_wrist_flex=2.0), 0.0, 10)
	assert controller.state == STATE_FOLLOWING
	held = pose(arm_wrist_flex=2.0)
	steps = run(controller, held, pose(arm_wrist_flex=40.0), 1.0, 30,
		motor=frozen_motor)
	assert all(state == STATE_FOLLOWING for _, _, _, state in steps)
	assert commands(steps, 'arm_wrist_flex') == pytest.approx([40.0] * 30)   # 목표 그대로 (막힌 관절도)
	assert 'arm_wrist_flex' in controller.warning
	# 경고는 상태를 바꾸지 않는다: 그친 적이 없으니 램프로 다시 시작하지도 않는다
	update(controller, 2.0, held, pose(arm_wrist_flex=40.0), 2.0)
	assert controller.state == STATE_FOLLOWING


def test_lagging_but_moving_arm_keeps_following() -> None:
	# 무거운 팔: 명령보다 4.5° 뒤처지지만 한 주기 1.5°씩 따라온다
	controller = make_controller()
	update(controller, 0.0, ZERO, ZERO, 0.0)
	steps = run(controller, ZERO, pose(arm_shoulder_lift=100.0), PERIOD, 60,
		motor=slow_motor(1.5))
	assert all(state == STATE_FOLLOWING for _, _, _, state in steps)
	assert controller.warning is None
	assert steps[-1][1]['arm_shoulder_lift'] == pytest.approx(88.5)


def test_blocked_gripper_and_small_static_error_keep_following() -> None:
	# 물건을 잡은 그리퍼는 막히는 것이 정상이다
	controller = make_controller()
	update(controller, 0.0, pose(arm_gripper=30.0), pose(arm_gripper=30.0), 0.0)
	steps = run(controller, pose(arm_gripper=30.0), ZERO, PERIOD, 90,
		motor=gripper_blocked_at(30.0))
	assert all(state == STATE_FOLLOWING for _, _, _, state in steps)
	assert commands(steps, GRIPPER)[-1] == pytest.approx(0.0)   # 목표 그대로 (모터가 스스로 힘을 줄인다)
	assert controller.warning is None
	# 중력으로 3° 처진 어깨 (track_tolerance_deg 4° 안)
	controller = make_controller()
	sagged = pose(arm_shoulder_lift=-3.0)
	steps = run(controller, sagged, ZERO, 0.0, 90, motor=frozen_motor)
	assert all(state == STATE_FOLLOWING for _, _, _, state in steps)
	assert controller.warning is None


def test_arm_not_ready_is_idle_and_ready_again_ramps_from_measured() -> None:
	# (c) 드라이버가 팔 명령을 받지 않으면 (arm_ready false) 목표가 와도 명령하지 않는다
	controller = make_controller()
	run(controller, ZERO, ZERO, 0.0, 5)
	goal = pose(arm_shoulder_pan=0.0)
	assert controller.update(1.0, ZERO, goal, 1.0, False) is None
	assert controller.state == STATE_IDLE
	assert controller.reason == '팔 토크가 꺼져 있습니다'
	assert controller.diffs == {}
	assert controller.update(2.0, ZERO, goal, 2.0, False) is None
	# 꺼진 동안 손으로 옮겼다: arm_ready가 다시 true가 되면 잰 자세에서 램프
	moved = pose(arm_shoulder_pan=40.0)
	command = controller.update(3.0, moved, goal, 3.0, True)
	assert controller.state == STATE_RAMPING
	assert command['arm_shoulder_pan'] == pytest.approx(40.0)


def test_halt_reports_error_then_restarts_from_follower() -> None:
	controller = make_controller()
	run(controller, ZERO, pose(arm_shoulder_pan=5.0), 0.0, 5)
	controller.halt('모터 상태가 없습니다', is_error=True)
	assert controller.state == STATE_ERROR
	assert controller.reason == '모터 상태가 없습니다'
	assert controller.diffs == {}
	follower = pose(arm_shoulder_pan=-40.0)
	command = update(controller, 1.0, follower, pose(arm_shoulder_pan=5.0), 1.0)
	assert controller.state == STATE_RAMPING
	assert command['arm_shoulder_pan'] == pytest.approx(-40.0)


def test_invalid_settings_raise() -> None:
	for settings in (
		{'ramp_time_s': 0.0},
		{'max_step_deg': -1.0},
		{'goal_timeout_s': 0.0},
		{'ramp_threshold_deg': -0.1},
		{'track_timeout_s': 0.0},
		{'track_tolerance_deg': 0.0},
		{'track_tolerance_deg': STEP},   # max_step_deg 이상이면 막힌 관절을 못 잡는다
	):
		with pytest.raises(ValueError):
			make_controller(**settings)


def test_commands_are_finite_and_complete() -> None:
	controller = make_controller()
	steps = run(controller, ZERO, pose(arm_wrist_roll=-170.0), 0.0, 90)
	assert len(steps) == 90
	for _, _, command, _ in steps:
		assert set(command) == set(ARM_JOINTS)
		assert all(math.isfinite(value) for value in command.values())


# ── 입력 검사 (lerobot 없이) ────────────────────────────
def test_lerobot_version_must_be_0_6() -> None:
	for version in ('0.6', '0.6.0', '0.6.3', '0.6.0.dev1'):
		assert lerobot_version_error(version) is None, version
	for version in ('0.7.0', '0.5.9', '0.60.1', '1.6.0', 'unknown', ''):
		reason = lerobot_version_error(version)
		assert reason is not None and '0.6' in reason, version


def motor(name: str, **values) -> SimpleNamespace:
	"""lekiwi_msgs/MotorState 대신 (online, 이상 없음, ID는 lerobot LeKiwi)."""
	fields = {
		'name': name, 'id': ARM_IDS.get(name, 7), 'online': True, 'status': 0,
		'position_ticks': 2048}
	return SimpleNamespace(**(fields | values))


def test_arm_motor_error_checks_ids_and_fault_bits() -> None:
	wheels = [motor('base_left_wheel', id=7, status=32)]   # 바퀴 이상은 팔과 상관없다
	healthy = [motor(name) for name in ARM_JOINTS] + wheels
	assert arm_motor_error(healthy) is None
	swapped = [motor(name, id=4) if name == 'arm_elbow_flex' else motor(name)
		for name in ARM_JOINTS]
	assert arm_motor_error(swapped) == (
		'팔 모터 ID가 다릅니다: arm_elbow_flex 4 (LeKiwi 3)')
	faulty = [
		motor(name, status=32) if name == 'arm_elbow_flex' else motor(name)
		for name in ARM_JOINTS]
	assert arm_motor_error(faulty) == (
		'팔 모터 이상으로 명령을 멈춥니다: arm_elbow_flex 과부하')
	several = [   # 16은 이상 비트가 아니다
		motor(name, status=1 | 4 | 16) if name == GRIPPER else motor(name)
		for name in ARM_JOINTS]
	assert arm_motor_error(several) == (
		'팔 모터 이상으로 명령을 멈춥니다: arm_gripper 전압 · 과열')
	for bit, label in ((2, '각도 센서'), (8, '과전류')):
		assert label in arm_motor_error([motor('arm_wrist_roll', status=bit)])
	assert arm_motor_error([motor('arm_wrist_roll', status=16)]) is None
	# 응답 없는 모터의 status는 0이다 (MotorState): 오프라인은 따로 본다
	offline = motor('arm_wrist_roll', online=False, status=32)
	assert arm_motor_error([offline]) is None
	# 그리퍼 과부하는 물건을 쥐면 생긴다: 오류가 아니다 (과열 등은 오류)
	assert arm_motor_error([motor(GRIPPER, status=32)]) is None
	assert arm_motor_error([motor(GRIPPER, status=32 | 8)]) == (
		'팔 모터 이상으로 명령을 멈춥니다: arm_gripper 과전류')


LATCH = '(이상이 풀린 뒤 원격 조작을 다시 시작하면 이어 갑니다)'


def lekiwi04_calibration() -> dict[str, tuple[int, int, int]]:
	data = json.loads(CALIBRATION.read_text(encoding='utf-8'))
	return {
		name: (data[name]['homing_offset'], data[name]['range_min'],
			data[name]['range_max'])
		for name in ARM_JOINTS}


def registers(**changes) -> SimpleNamespace:
	"""lekiwi04 모터를 그대로 읽은 lekiwi_msgs/ArmRegisters. changes는 필드 → {ID: 값}."""
	calibration = lekiwi04_calibration()
	fields = {
		'ids': [ARM_IDS[name] for name in ARM_JOINTS],
		'valid': [True] * 6,
		'homing_offset': [calibration[name][0] for name in ARM_JOINTS],
		'min_position_limit': [calibration[name][1] for name in ARM_JOINTS],
		'max_position_limit': [calibration[name][2] for name in ARM_JOINTS],
		'p_coefficient': [16] * 6,
		'i_coefficient': [0] * 6,
		'd_coefficient': [32] * 6,
	}
	for field, values in changes.items():
		for motor_id, value in values.items():
			fields[field][motor_id - 1] = value
	return SimpleNamespace(**fields)


def test_registers_match_calibration_like_lerobot_is_calibrated() -> None:
	calibration = lekiwi04_calibration()
	assert compare_registers(registers(), calibration, 'lekiwi04') == (
		None, None)
	error, warning = compare_registers(
		registers(homing_offset={6: -470}, max_position_limit={3: 3100}),
		calibration, 'lekiwi04')
	assert error == (
		'보정 파일이 이 로봇의 모터와 다릅니다 (lekiwi04.json): '
		'arm_elbow_flex range_max 모터 3100 · 파일 3094, '
		'arm_gripper homing_offset 모터 -470 · 파일 -473')
	assert warning is None
	error, _ = compare_registers(
		registers(valid={2: False}, min_position_limit={2: 0}),
		calibration, 'lekiwi04')
	assert error.endswith('(lekiwi04.json): arm_shoulder_lift 읽지 못함')
	missing = registers()
	for field in vars(missing):
		setattr(missing, field, getattr(missing, field)[:5])   # 그리퍼(6)가 없다
	error, _ = compare_registers(missing, calibration, 'lekiwi04')
	assert error.endswith('arm_gripper 읽지 못함')
	broken = registers()
	broken.valid = broken.valid[:3]
	error, _ = compare_registers(broken, calibration, 'lekiwi04')
	assert '길이' in error


def test_registers_pid_difference_is_only_a_warning() -> None:
	error, warning = compare_registers(
		registers(p_coefficient={1: 32}, d_coefficient={6: 0}),
		lekiwi04_calibration(), 'lekiwi04')
	assert error is None
	assert warning == (
		'팔 모터 PID가 lerobot LeKiwi(P 16 · I 0 · D 32)와 다릅니다: '
		'arm_shoulder_pan P 32 · I 0 · D 32, arm_gripper P 16 · I 0 · D 0')


def test_limit_ticks_clamps_to_measured() -> None:
	measured = {'arm_shoulder_pan': 2000, GRIPPER: 1700}
	ticks = {'arm_shoulder_pan': 2050, GRIPPER: 1970}
	assert limit_ticks(ticks, measured, 114) == {
		'arm_shoulder_pan': 2050, GRIPPER: 1814}
	assert limit_ticks({'arm_shoulder_pan': 1000}, measured, 114) == {
		'arm_shoulder_pan': 1886}


def test_make_status_rounds_and_lists_warnings() -> None:
	assert make_status(STATE_ERROR, '보정 파일이 없습니다: x', 'lekiwi04') == {
		'state': STATE_ERROR, 'reason': '보정 파일이 없습니다: x',
		'robot_id': 'lekiwi04', 'diffs': {}, 'ramp_progress': None,
		'warnings': []}
	status = make_status(
		STATE_RAMPING, 'r', 'lekiwi04', {'arm_gripper': 1.23456}, 0.61234,
		('w',))
	assert status['diffs'] == {'arm_gripper': 1.23}
	assert status['ramp_progress'] == 0.612
	assert status['warnings'] == ['w']


# ── FollowerArmLogic (노드의 판단, lerobot 없이) ─────────
class FakeConverter:
	"""ArmConverter 대신: lekiwi04 보정에 lerobot 공식을 그대로 쓴다 (lerobot 없이)."""

	def __init__(self) -> None:
		self.ranges = calibration_ranges()
		self.homing_offsets = {
			name: offset
			for name, (offset, _, _) in lekiwi04_calibration().items()}
		self.limits = {}
		self.units_per_degree = {}
		for name, (range_min, range_max) in self.ranges.items():
			ends = sorted(
				plain_units(name, tick, self.ranges)
				for tick in (range_min, range_max))
			self.limits[name] = (ends[0], ends[1])
			range_degrees = (range_max - range_min) * 360.0 / MAX_TICK
			self.units_per_degree[name] = (ends[1] - ends[0]) / range_degrees

	def to_units(self, ticks: dict[str, int]) -> Pose:
		return {
			name: plain_units(name, tick, self.ranges)
			for name, tick in ticks.items()}

	def to_ticks(self, units: Pose) -> dict[str, int]:
		ticks = {}
		for name, value in units.items():
			range_min, range_max = self.ranges[name]
			# lerobot _unnormalize: 그리퍼 RANGE_0_100 · 몸 관절 DEGREES, int로 버림
			if name == GRIPPER:
				bounded = min(100.0, max(0.0, value))
				span = range_max - range_min
				tick = int(bounded / 100.0 * span + range_min)
			else:
				middle = (range_min + range_max) / 2.0
				tick = int(value * MAX_TICK / 360.0 + middle)
			ticks[name] = min(MAX_TICK, max(0, tick))
		return ticks


def test_fake_converter_matches_lerobot(converter: ArmConverter) -> None:
	fake = FakeConverter()
	for name in ARM_JOINTS:
		assert fake.limits[name] == pytest.approx(converter.limits[name])
		assert fake.units_per_degree[name] == pytest.approx(
			converter.units_per_degree[name])
		for value in (-200.0, -37.77, -0.4, 0.0, 0.4, 12.345, 99.9, 150.0):
			units = {name: value}
			assert fake.to_ticks(units) == converter.to_ticks(units)
		for tick in (0, 1700, 2047, 3190, MAX_TICK):
			assert fake.to_units({name: tick})[name] == pytest.approx(
				converter.to_units({name: tick})[name])


# 보정 범위 가운데 (몸 관절 약 0°, 그리퍼 50)
HOME_TICKS = {
	name: (range_min + range_max) // 2
	for name, (range_min, range_max) in calibration_ranges().items()}
STEP_TICKS = 114   # 10° = 113.75 tick, 반올림


def make_logic(**settings: float) -> FollowerArmLogic:
	converter = FakeConverter()
	controller = FollowerController(
		converter.limits, converter.units_per_degree, **settings)
	return FollowerArmLogic(converter, controller, 'lekiwi04', 0.5)


def arm_states(ticks: dict[str, int], **joints: dict) -> list[SimpleNamespace]:
	"""팔 6개 + 바퀴 3개 모터 상태. joints는 관절 → 바꿀 필드."""
	states = [
		motor(name, position_ticks=ticks[name], **joints.get(name, {}))
		for name in ARM_JOINTS]
	return states + [motor(f'base_{side}_wheel', id=7 + index)
		for index, side in enumerate(('left', 'back', 'right'))]


def home_goal(logic: FollowerArmLogic, now: float, **ticks: int) -> None:
	"""HOME_TICKS (일부를 바꾼) 자세를 LeRobot 단위 목표로 준다."""
	units = FakeConverter().to_units(HOME_TICKS | ticks)
	assert logic.on_goal(now, list(units), list(units.values()))


def ready_logic() -> FollowerArmLogic:
	"""모터 상태 · 레지스터 · arm_ready가 다 온 판단 (t = 0, 가운데 자세)."""
	logic = make_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS))
	logic.on_registers(registers())
	logic.on_arm_ready(True)
	return logic


def test_logic_needs_states_registers_and_arm_ready() -> None:
	logic = make_logic()
	assert logic.step(0.0) is None
	assert logic.controller.state == STATE_ERROR
	assert '모터 상태가 0.5초' in logic.controller.reason
	follower = logic.on_motor_states(0.0, arm_states(HOME_TICKS))
	assert follower['arm_shoulder_pan'] == pytest.approx(0.0, abs=0.1)
	assert follower[GRIPPER] == pytest.approx(50.0, abs=0.1)
	home_goal(logic, 0.0)
	assert logic.step(0.01) is None   # 레지스터를 아직 못 받았다
	assert logic.controller.state == STATE_IDLE
	assert 'arm_registers' in logic.controller.reason
	assert logic.on_registers(registers()) == (None, None)
	assert logic.step(0.02) is None   # 드라이버가 아직 팔 명령을 받지 않는다
	assert logic.controller.reason == '팔 토크가 꺼져 있습니다'
	logic.on_arm_ready(True)
	ticks = logic.step(0.03)
	assert logic.controller.state == STATE_FOLLOWING
	assert ticks == pytest.approx(HOME_TICKS, abs=1)
	assert logic.status()['state'] == STATE_FOLLOWING
	assert logic.status()['warnings'] == []


def test_logic_rejects_incomplete_or_nan_goal() -> None:
	logic = ready_logic()
	assert not logic.on_goal(0.0, list(ARM_JOINTS[:5]), [0.0] * 5)
	assert not logic.on_goal(0.0, list(ARM_JOINTS), [0.0] * 5 + [math.nan])
	assert logic.step(0.01) is None
	assert logic.controller.reason == '목표를 기다립니다'


def test_arm_ready_false_holds_and_true_ramps_from_measured() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	assert logic.controller.state == STATE_FOLLOWING
	logic.on_arm_ready(False)   # 비상 정지 · 팔 힘 빼기 · 드라이버 재시작
	home_goal(logic, 0.05)
	assert logic.step(0.05) is None
	assert logic.controller.state == STATE_IDLE
	assert logic.controller.reason == '팔 토크가 꺼져 있습니다'
	# 토크가 꺼진 동안 팔이 떨어졌다 (pan +600 tick ≈ 53°)
	dropped = HOME_TICKS | {
		'arm_shoulder_pan': HOME_TICKS['arm_shoulder_pan'] + 600}
	logic.on_motor_states(1.0, arm_states(dropped))
	home_goal(logic, 1.0)
	logic.on_arm_ready(True)
	ticks = logic.step(1.0)
	assert logic.controller.state == STATE_RAMPING
	assert logic.controller.ramp_progress == 0.0
	# 잰 자세에서 출발
	assert ticks['arm_shoulder_pan'] == dropped['arm_shoulder_pan']
	home_goal(logic, 1.5)
	ticks = logic.step(1.5)
	assert logic.controller.ramp_progress == pytest.approx(0.25)
	assert HOME_TICKS['arm_shoulder_pan'] < ticks['arm_shoulder_pan'] < (
		dropped['arm_shoulder_pan'])


def test_brief_offline_skips_cycle_and_keeps_the_ramp() -> None:
	logic = ready_logic()
	start = HOME_TICKS | {'arm_elbow_flex': HOME_TICKS['arm_elbow_flex'] + 400}
	logic.on_motor_states(0.0, arm_states(start))
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	assert logic.controller.state == STATE_RAMPING
	# 팔꿈치 읽기 한 번 실패 (드라이버 30 Hz)
	missed = arm_states(start, arm_elbow_flex={'online': False})
	assert logic.on_motor_states(0.5, missed) is None
	home_goal(logic, 0.5)
	assert logic.step(0.5) is None
	assert logic.controller.state == STATE_RAMPING   # 오류가 아니고 램프도 그대로
	assert logic.status()['warnings'] == [
		'팔 모터 응답이 잠깐 끊겨 이번 주기는 쉽니다: arm_elbow_flex']
	logic.on_motor_states(0.55, arm_states(start))
	home_goal(logic, 0.55)
	assert logic.step(0.55) is not None
	assert logic.controller.state == STATE_RAMPING
	assert logic.controller.ramp_progress == pytest.approx(0.275)   # 처음부터가 아니다
	assert logic.warnings == []


def test_persistent_offline_halts_with_error_then_restarts() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	offline = {GRIPPER: {'online': False}}
	for cycle in range(10):   # 0.0 ~ 0.45 s: 아직 쉬기만 한다
		now = 0.05 * (cycle + 1)
		logic.on_motor_states(now, arm_states(HOME_TICKS, **offline))
		home_goal(logic, now)
		assert logic.step(now) is None
		assert logic.controller.state == STATE_FOLLOWING, now
	logic.on_motor_states(0.55, arm_states(HOME_TICKS, **offline))
	home_goal(logic, 0.55)
	assert logic.step(0.55) is None   # 처음 끊긴 0.05 s에서 0.5 s
	assert logic.controller.state == STATE_ERROR
	assert logic.controller.reason == '팔 모터가 응답하지 않습니다: arm_gripper'
	logic.on_motor_states(0.6, arm_states(HOME_TICKS))
	home_goal(logic, 0.6)
	assert logic.step(0.6) is not None   # 돌아오면 잰 자세에서 새로 출발
	assert logic.controller.state == STATE_FOLLOWING


def test_stale_states_are_an_error() -> None:
	logic = ready_logic()
	home_goal(logic, 0.5)
	assert logic.step(0.5) is not None
	home_goal(logic, 0.51)
	assert logic.step(0.51) is None
	assert logic.controller.state == STATE_ERROR
	assert 'lekiwi_motor_driver_node' in logic.controller.reason


def test_fault_holds_once_then_latches_until_goals_pause() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	logic.on_motor_states(
		0.05, arm_states(HOME_TICKS, arm_elbow_flex={'status': 32}))
	home_goal(logic, 0.05)
	# 걸린 순간 잰 자세를 한 번 보낸다: 막힌 관절이 마지막 목표로 계속 밀지 않게
	assert logic.step(0.05) == HOME_TICKS
	assert logic.controller.state == STATE_ERROR
	assert logic.controller.reason == (
		f'팔 모터 이상으로 명령을 멈춥니다: arm_elbow_flex 과부하 {LATCH}')
	home_goal(logic, 0.06)
	assert logic.step(0.06) is None
	# 이상이 풀려도 리더 목표가 이어 오는 동안은 멈춰 있다 (과부하 · 재시도 되풀이 없음)
	for index in range(10):
		now = 0.1 + 0.05 * index
		logic.on_motor_states(now, arm_states(HOME_TICKS))
		home_goal(logic, now)
		assert logic.step(now) is None
		assert logic.controller.state == STATE_ERROR
	# 사람이 원격 조작을 멈췄다 (목표가 0.5초 넘게 끊김): 풀린다. 팔 토크는 그대로다
	logic.on_motor_states(1.1, arm_states(HOME_TICKS))
	assert logic.step(1.1) is None
	assert logic.controller.state == STATE_IDLE
	home_goal(logic, 1.15)
	assert logic.step(1.15) is not None


def test_fault_still_present_keeps_the_latch_after_goals_pause() -> None:
	logic = ready_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS, arm_wrist_flex={'status': 4}))
	assert logic.step(0.0) == HOME_TICKS
	logic.on_motor_states(1.0, arm_states(HOME_TICKS, arm_wrist_flex={'status': 4}))
	home_goal(logic, 1.0)
	assert logic.step(1.0) is None
	assert '과열' in logic.controller.reason


def test_fault_seen_while_arm_off_does_not_block_next_enable() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	logic.on_motor_states(
		0.05, arm_states(HOME_TICKS, arm_elbow_flex={'status': 4}))
	logic.step(0.05)
	logic.on_arm_ready(False)   # 사람이 팔 토크를 끈다
	# 꺼진 뒤에도 비트가 잠깐 남아 있다 (오류로 보이지만 걸어 두지 않는다)
	logic.on_motor_states(
		0.1, arm_states(HOME_TICKS, arm_elbow_flex={'status': 4}))
	assert logic.step(0.1) is None
	assert logic.controller.state == STATE_ERROR
	logic.on_motor_states(0.15, arm_states(HOME_TICKS))
	logic.on_arm_ready(True)
	home_goal(logic, 0.15)
	assert logic.step(0.15) is not None
	assert logic.controller.state == STATE_FOLLOWING


def test_repeated_arm_ready_restarts_from_measured_pose() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	assert logic.controller.state == STATE_FOLLOWING
	# false → true가 이어 와 false를 놓쳤다: true만 다시 와도 잰 자세에서 램프 여부를 다시 본다
	logic.on_arm_ready(True)
	logic.on_motor_states(0.05, arm_states(HOME_TICKS))
	home_goal(logic, 0.05, arm_shoulder_pan=HOME_TICKS['arm_shoulder_pan'] + 400)
	assert logic.step(0.05) is not None
	assert logic.controller.state == STATE_RAMPING


def test_gripper_overload_is_a_warning() -> None:
	logic = ready_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS, arm_gripper={'status': 32}))
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	assert logic.controller.state == STATE_FOLLOWING
	assert any('그리퍼 과부하' in warning for warning in logic.warnings)
	# 그리퍼라도 과열은 오류다
	logic.on_motor_states(0.05, arm_states(HOME_TICKS, arm_gripper={'status': 32 | 4}))
	home_goal(logic, 0.05)
	assert logic.step(0.05) == HOME_TICKS
	assert logic.controller.reason == f'팔 모터 이상으로 명령을 멈춥니다: arm_gripper 과열 {LATCH}'


def test_wrong_ids_halt_with_error() -> None:
	logic = ready_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS, arm_wrist_flex={'id': 5}))
	home_goal(logic, 0.0)
	assert logic.step(0.0) is None   # 잡기(hold)도 보내지 않는다: ID가 틀리면 다른 모터에 간다
	assert logic.controller.state == STATE_ERROR
	assert logic.controller.reason == f'팔 모터 ID가 다릅니다: arm_wrist_flex 5 (LeKiwi 4) {LATCH}'
	# 팔 토크를 껐다 켜도 다음 모터 상태를 기다리지 않고 막는다 (틈에 다른 모터의 tick을 보내지 않게)
	logic.on_arm_ready(False)
	logic.on_arm_ready(True)
	home_goal(logic, 0.01)
	assert logic.step(0.01) is None
	assert logic.controller.state == STATE_ERROR


def test_registers_mismatch_is_an_error_and_pid_a_warning() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	error, warning = logic.on_registers(registers(homing_offset={1: 0}))
	assert error.startswith('보정 파일이 이 로봇의 모터와 다릅니다 (lekiwi04.json)')
	assert logic.step(0.0) is None
	assert logic.controller.state == STATE_ERROR
	assert logic.controller.reason == error
	logic.on_registers(registers(valid={4: False}))
	assert logic.step(0.01) is None
	assert 'arm_wrist_flex 읽지 못함' in logic.controller.reason
	error, warning = logic.on_registers(registers(p_coefficient={6: 32}))
	assert error is None and 'arm_gripper P 32' in warning
	assert logic.step(0.02) is not None
	assert logic.status()['warnings'] == [warning]


def test_publish_clamps_out_of_range_gripper_to_measured_step() -> None:
	# 그리퍼가 보정 범위(1970~3586) 밖 1700에 있다: lerobot은 0으로 읽는다
	logic = ready_logic()
	low = HOME_TICKS | {GRIPPER: 1700}
	assert logic.on_motor_states(0.0, arm_states(low))[GRIPPER] == 0.0
	home_goal(logic, 0.0, arm_gripper=1970)   # 그리퍼 목표 0 (범위 끝)
	ticks = logic.step(0.0)
	assert logic.controller.state == STATE_FOLLOWING   # 단위로는 차이가 0이다
	assert ticks[GRIPPER] == 1700 + STEP_TICKS          # 1970으로 튀지 않는다
	assert all(
		abs(ticks[name] - low[name]) <= STEP_TICKS for name in ARM_JOINTS)


def test_logic_tracking_warning_does_not_stop_commands() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) is not None
	far = HOME_TICKS['arm_shoulder_pan'] + 50   # 약 4.4°: 램프 없이 따라간다
	for cycle in range(1, 15):   # 팔이 막혀 움직이지 않는다
		now = cycle / 30.0
		logic.on_motor_states(now, arm_states(HOME_TICKS))
		home_goal(logic, now, arm_shoulder_pan=far)
		ticks = logic.step(now)
		assert ticks['arm_shoulder_pan'] == far
	assert logic.controller.state == STATE_FOLLOWING
	assert any('따라오지' in line for line in logic.status()['warnings'])


def test_logic_rejects_bad_state_timeout() -> None:
	converter = FakeConverter()
	controller = FollowerController(
		converter.limits, converter.units_per_degree)
	with pytest.raises(ValueError):
		FollowerArmLogic(converter, controller, 'lekiwi04', 0.0)


def test_logic_confirms_the_arm_only_with_lekiwi_ids_and_matching_registers() -> None:
	logic = make_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS))
	assert not logic.is_arm_confirmed   # 레지스터를 아직 못 받았다
	logic.on_registers(registers(min_position_limit={3: 0}))   # 다른 로봇의 보정 파일
	assert not logic.is_arm_confirmed
	logic.on_registers(registers())
	assert logic.is_arm_confirmed
	# 팔 모터 ID가 lerobot LeKiwi와 다르다 (드라이버 motor_ids 설정 실수)
	logic.on_motor_states(0.05, arm_states(HOME_TICKS, arm_elbow_flex={'id': 4}))
	assert not logic.is_arm_confirmed
	logic.on_motor_states(0.1, arm_states(HOME_TICKS))
	assert logic.is_arm_confirmed


# ── lerobot 관측 상태: 바퀴 → 몸체 속도 (lerobot _wheel_raw_to_body) ──
STEPS_PER_DEG = 4096.0 / 360.0


def wheel_states(raw: list[int], **changes: dict) -> list[SimpleNamespace]:
	"""바퀴 3개 모터 상태. velocity는 드라이버처럼 tick × 2π / 4096 [rad/s] (sts3215 TicksToRad)."""
	return [
		SimpleNamespace(**({
			'name': name, 'id': motor_id, 'online': True,
			'velocity': tick * (2.0 * math.pi) / 4096} | changes.get(name, {})))
		for (name, motor_id), tick in zip(BASE_WHEELS.items(), raw)]


def body_to_wheel_raw(x: float, y: float, theta_degps: float) -> list[float]:
	"""lerobot 정기구학을 numpy 없이 다시 쓴 것 (자르기 · 반올림 없음): 몸체 속도 → 바퀴 [tick/s]."""
	theta = math.radians(theta_degps)
	return [
		(math.cos(a) * x + math.sin(a) * y + 0.125 * theta) / 0.05 * 180.0 / math.pi * STEPS_PER_DEG
		for a in (math.radians(angle - 90) for angle in (240, 0, 120))]


def test_lerobot_state_names_are_the_lekiwi_dataset_keys() -> None:
	assert LEROBOT_STATE_NAMES == (
		'arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
		'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos',
		'x.vel', 'y.vel', 'theta.vel')
	assert BASE_WHEELS == {'base_left_wheel': 7, 'base_back_wheel': 8, 'base_right_wheel': 9}


def test_wheel_raw_to_body_inverts_the_lerobot_forward_kinematics() -> None:
	# 세 바퀴가 같으면 제자리 회전: theta = tick × 360/4096 × 0.05/0.125 [deg/s]
	assert wheel_raw_to_body(1000, 1000, 1000) == pytest.approx(
		(0.0, 0.0, 1000 * 360.0 / 4096 * 0.05 / 0.125), abs=1e-12)
	rng = random.Random(7)
	for _ in range(200):
		body = (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-120.0, 120.0))
		assert wheel_raw_to_body(*body_to_wheel_raw(*body)) == pytest.approx(body, abs=1e-9)


def test_base_velocity_uses_driver_ticks_and_needs_all_wheels() -> None:
	raw = [123, -2500, 3000]
	assert base_velocity(wheel_states(raw)) == wheel_raw_to_body(*raw)
	assert base_velocity(wheel_states([0, 0, 0])) == (0.0, 0.0, 0.0)
	# 팔 · 다른 모터가 섞여도 이름으로 고른다
	assert base_velocity(arm_states(HOME_TICKS)[:6] + wheel_states(raw)) == wheel_raw_to_body(*raw)
	for name in BASE_WHEELS:
		assert base_velocity(wheel_states(raw, **{name: {'online': False}})) is None
		assert base_velocity(wheel_states(raw, **{name: {'velocity': math.nan}})) is None
		assert base_velocity([m for m in wheel_states(raw) if m.name != name]) is None
	assert base_velocity(wheel_states(raw, base_back_wheel={'id': 9})) is None


def test_base_velocity_matches_lerobot_wheel_raw_to_body() -> None:
	"""lerobot이 같은 Present_Velocity로 내는 값과 1e-9 안에서 같다 (드라이버 rad/s를 거쳐도)."""
	lekiwi = pytest.importorskip('lerobot.robots.lekiwi.lekiwi')
	rng = random.Random(20261022)
	for limit in (3000, 0x7FFF):   # 드라이버 · lerobot 바퀴 상한, 부호-크기 전체
		for _ in range(1000):
			raw = [rng.randint(-limit, limit) for _ in BASE_WHEELS]
			want = lekiwi.LeKiwi._wheel_raw_to_body(lekiwi.LeKiwi, *raw)
			assert base_velocity(wheel_states(raw)) == pytest.approx(
				(want['x.vel'], want['y.vel'], want['theta.vel']), abs=1e-9, rel=0), raw


# ── 접은 자세 (lekiwi/arm/rest) ─────────────────────────
REST_TICKS = [1787, 1230, 3190, 2636, 3235, 2002]   # lekiwi04 접은 자세, ID 1~6 (팔꿈치는 보정 범위 3094 밖)
REST = dict(zip(ARM_JOINTS, REST_TICKS))
ELBOW_MAX = calibration_ranges()['arm_elbow_flex'][1]   # 3094


def rest_logic(**settings: float) -> FollowerArmLogic:
	"""접은 자세가 있고 모터 상태 · 레지스터 · arm_ready가 다 온 판단 (t = 0, 가운데 자세)."""
	converter = FakeConverter()
	controller = FollowerController(converter.limits, converter.units_per_degree, **settings)
	logic = FollowerArmLogic(converter, controller, 'lekiwi04', 0.5, REST_TICKS)
	logic.on_motor_states(0.0, arm_states(HOME_TICKS))
	logic.on_registers(registers())
	logic.on_arm_ready(True)
	return logic


def ideal_servo(measured: dict[str, int], command: dict[str, int] | None) -> dict[str, int]:
	return measured if command is None else measured | command


Ticks = dict[str, int]


def run_logic(
	logic: FollowerArmLogic,
	start: float,
	cycles: int,
	motor: Callable[[Ticks, Ticks | None], Ticks] = ideal_servo,
	ticks: Ticks = HOME_TICKS,
	goal: dict[str, int] | None = None,
) -> list[tuple[float, Ticks, Ticks | None, str]]:
	"""30 Hz로 돈다 (goal이 있으면 주기마다 리더 목표 tick도 준다): (시각, 잰 tick, 명령 tick, 상태) 목록."""
	steps = []
	measured = dict(ticks)
	for cycle in range(cycles):
		now = start + cycle * PERIOD
		logic.on_motor_states(now, arm_states(measured))
		if goal is not None:
			home_goal(logic, now, **goal)
		command = logic.step(now)
		steps.append((now, dict(measured), command, logic.controller.state))
		measured = motor(measured, command)
	return steps


def test_rest_is_refused_until_the_follower_can_command() -> None:
	assert make_logic().rest(0.0) is not None   # 모터 상태가 없다
	no_pose = ready_logic()
	assert no_pose.rest(0.0) == '접은 자세가 설정되지 않았습니다 (rest_pose_ticks)'
	logic = rest_logic()
	logic.on_arm_ready(False)
	assert logic.rest(0.0) == '팔 토크가 꺼져 있습니다'
	logic = rest_logic()
	logic.on_registers(registers(min_position_limit={3: 0}))   # 다른 로봇의 보정 파일
	assert logic.rest(0.0).startswith('보정 파일이 이 로봇의 모터와 다릅니다')
	logic = rest_logic()
	logic.on_motor_states(0.05, arm_states(HOME_TICKS, arm_elbow_flex={'status': 32}))
	assert logic.rest(0.05) == f'팔 모터 이상으로 명령을 멈춥니다: arm_elbow_flex 과부하 {LATCH}'
	logic = rest_logic()
	assert '모터 상태가 0.5초' in logic.rest(1.0)
	# 거절하면 아무것도 바뀌지 않는다
	assert logic.controller.state not in (STATE_RESTING, STATE_RESTED)
	logic.on_motor_states(1.0, arm_states(HOME_TICKS))
	assert logic.step(1.0) is None
	# 레지스터 없음
	logic = make_logic()
	logic.on_motor_states(0.0, arm_states(HOME_TICKS))
	logic.on_arm_ready(True)
	assert 'arm_registers' in logic.rest(0.0)


def test_rest_pose_ticks_must_be_six_motor_ticks() -> None:
	converter = FakeConverter()
	controller = FollowerController(converter.limits, converter.units_per_degree)
	for ticks in (
		REST_TICKS[:5], REST_TICKS[:5] + [4096], REST_TICKS[:5] + [-1],
		[float(tick) for tick in REST_TICKS], REST_TICKS[:5] + [True], 'abcdef', 1787,
	):
		with pytest.raises(ValueError, match='rest_pose_ticks는'):
			FollowerArmLogic(converter, controller, 'lekiwi04', 0.5, ticks)
	# 비우면 접은 자세 없음 (rclpy 정수 배열은 array.array로 온다)
	assert FollowerArmLogic(converter, controller, 'lekiwi04', 0.5, ()).rest(0.0) is not None
	# 게이트웨이 arm_profile.yaml은 지울 때 ''를 쓴다 (rcl이 뒤 파일의 []를 무시해서)
	assert FollowerArmLogic(converter, controller, 'lekiwi04', 0.5, '').rest(0.0) is not None
	FollowerArmLogic(converter, controller, 'lekiwi04', 0.5, array.array('q', REST_TICKS))
	for settings in ({'rest_time_s': 0.0}, {'rest_tolerance_deg': 0.0}, {'rest_tolerance_deg': 10.0}):
		with pytest.raises(ValueError):
			FollowerController(converter.limits, converter.units_per_degree, **settings)


def test_rest_ramps_from_measured_with_smoothstep_over_rest_time() -> None:
	logic = rest_logic()
	assert logic.rest(0.0) is None
	assert logic.controller.state == STATE_RESTING
	assert logic.status()['ramp_progress'] == 0.0
	steps = run_logic(logic, 0.0, 90)   # 3 s
	converter = FakeConverter()
	start = converter.to_units(HOME_TICKS)
	target = {name: min(high, max(low, value)) for name, value in converter.to_units(REST).items()
		for low, high in (converter.limits[name],)}
	pans = []
	for now, _, command, state in steps:
		progress = now / 3.0
		weight = progress * progress * (3.0 - 2.0 * progress)
		want = start['arm_shoulder_pan'] + weight * (target['arm_shoulder_pan'] - start['arm_shoulder_pan'])
		assert command['arm_shoulder_pan'] == pytest.approx(converter.to_ticks({'arm_shoulder_pan': want})['arm_shoulder_pan'], abs=1)
		assert state == STATE_RESTING
		pans.append(command['arm_shoulder_pan'])
	# 처음 · 끝은 느리고 가운데가 빠르다, 늘 한 방향 (pan 2087 → 1787)
	assert all(b <= a for a, b in zip(pans, pans[1:]))
	assert pans[0] - pans[2] < pans[44] - pans[46]
	assert pans[86] - pans[88] < pans[44] - pans[46]
	progress = [step[0] / 3.0 for step in steps]
	assert logic.status()['ramp_progress'] == pytest.approx(progress[-1], abs=1e-3)


def test_rest_keeps_the_step_limit_to_the_measured_pose() -> None:
	# 램프가 짧아 목표가 한 주기에 10°보다 멀리 가도, 명령은 잰 tick ± 114 tick 안이다
	logic = rest_logic(rest_time_s=0.1)
	assert logic.rest(0.0) is None
	steps = run_logic(logic, 0.0, 30, motor=slow_servo(40))
	for _, measured, command, _ in steps:
		if command is not None:
			assert all(abs(command[name] - measured[name]) <= STEP_TICKS for name in ARM_JOINTS)
	assert any(abs(steps[3][2][name] - steps[3][1][name]) == STEP_TICKS for name in ARM_JOINTS)


def slow_servo(ticks_per_cycle: int) -> Callable[[Ticks, Ticks | None], Ticks]:
	"""명령을 받지만 한 주기에 ticks_per_cycle만 간다."""
	def motor(measured: dict[str, int], command: dict[str, int] | None) -> dict[str, int]:
		if command is None:
			return measured
		return {
			name: value + min(ticks_per_cycle, max(-ticks_per_cycle, command[name] - value))
			for name, value in measured.items()}
	return motor


def test_rest_clips_the_folded_elbow_and_reports_rested_within_tolerance() -> None:
	logic = rest_logic()
	assert logic.rest(0.0) is None
	steps = run_logic(logic, 0.0, 100)
	commanded = [command for _, _, command, _ in steps if command is not None]
	# 팔꿈치는 보정 범위 끝(3094)에서 멈춘다 (접은 자세 3190은 범위 밖). 나머지는 접은 자세 그대로
	assert max(command['arm_elbow_flex'] for command in commanded) <= ELBOW_MAX
	assert commanded[-1] == pytest.approx(REST | {'arm_elbow_flex': ELBOW_MAX}, abs=1)
	# 램프가 끝난 첫 주기에 rested (이상적인 모터는 다 와 있다). 그 주기에 마지막 명령을 보내고 그친다
	rested = [index for index, step in enumerate(steps) if step[3] == STATE_RESTED]
	assert steps[rested[0]][0] == pytest.approx(3.0, abs=PERIOD)
	assert steps[rested[0]][2] is not None
	assert all(step[2] is None for step in steps[rested[0] + 1:])
	status = logic.status()
	assert status['state'] == STATE_RESTED
	assert status['ramp_progress'] is None
	assert status['warnings'] == []
	assert abs(status['diffs']['arm_elbow_flex']) <= 3.0
	# 이미 접은 자세면 램프 없이 바로 rested
	logic = rest_logic()
	folded = REST | {'arm_elbow_flex': ELBOW_MAX}
	logic.on_motor_states(0.0, arm_states(folded))
	assert logic.rest(0.0) is None
	assert logic.controller.reason == '이미 접은 자세입니다'
	assert logic.step(0.0) is not None
	assert logic.controller.state == STATE_RESTED


def test_rest_from_the_real_fold_does_not_unfold_the_elbow_to_the_range_edge() -> None:
	# 실물 접은 자세: 팔꿈치 3190이 보정 범위(3094) 밖. 범위 끝까지 8° 펴지 않고 그 자리에 둔다
	logic = rest_logic()
	steps = run_logic(logic, 0.0, 1, ticks=REST)
	assert logic.rest(PERIOD) is None
	steps = run_logic(logic, PERIOD, 40, ticks=REST)
	commanded = [command for _, _, command, _ in steps if command is not None]
	assert commanded and all(abs(command['arm_elbow_flex'] - 3190) <= 1 for command in commanded)
	assert logic.controller.state == STATE_RESTED
	assert logic.status()['warnings'] == []
	# 범위 밖 반대쪽이거나 범위 안이면 범위 끝까지 간다 (위 clips 시험)


def test_rested_with_a_warning_when_a_joint_does_not_arrive() -> None:
	def wrist_stuck(measured: Ticks, command: Ticks | None) -> Ticks:   # 손목이 걸려 움직이지 않는다
		moved = ideal_servo(measured, command)
		return moved | {'arm_wrist_flex': measured['arm_wrist_flex']}

	logic = rest_logic()
	assert logic.rest(0.0) is None
	steps = run_logic(logic, 0.0, 140, motor=wrist_stuck)
	states = {round(now, 3): state for now, _, _, state in steps}
	# 램프가 끝나고 (3 s) 1초 동안은 아직 resting (ramp_progress 1.0), 그 뒤 경고와 함께 rested
	assert all(state == STATE_RESTING for now, state in states.items() if now < 3.99)
	assert all(state == STATE_RESTED for now, state in states.items() if now > 4.01)
	status = logic.status()
	assert status['state'] == STATE_RESTED
	assert status['warnings'] == ['접은 자세에 다 오지 못했습니다: arm_wrist_flex']
	# rested 뒤에는 명령을 내지 않는다 (드라이버가 마지막 목표를 유지한다)
	assert all(command is None for now, _, command, _ in steps if now > 4.05)


def test_leader_goals_are_ignored_while_resting() -> None:
	logic = rest_logic()
	run_logic(logic, 0.0, 10, goal={})   # 리더를 따라가는 중
	assert logic.controller.state == STATE_FOLLOWING
	assert logic.rest(10 * PERIOD) is None
	far = {'arm_shoulder_pan': 3400, 'arm_elbow_flex': 900}
	steps = run_logic(logic, 10 * PERIOD, 120, goal=far)
	commanded = [command for _, _, command, _ in steps if command is not None]
	assert all(command['arm_shoulder_pan'] <= HOME_TICKS['arm_shoulder_pan'] for command in commanded)
	assert all(command['arm_elbow_flex'] >= HOME_TICKS['arm_elbow_flex'] for command in commanded)
	assert commanded[-1] == pytest.approx(REST | {'arm_elbow_flex': ELBOW_MAX}, abs=1)
	assert steps[-1][3] == STATE_RESTED


def test_any_arm_ready_message_or_a_fault_ends_rest_mode() -> None:
	for is_ready in (True, False):   # 토크 끔, 또는 같은 값이 다시 와도
		logic = rest_logic()
		assert logic.rest(0.0) is None
		run_logic(logic, 0.0, 30)
		logic.on_arm_ready(is_ready)
		assert logic.controller.state == STATE_IDLE
		steps = run_logic(logic, 1.0, 5, goal={})
		assert all(state != STATE_RESTING for *_, state in steps)
		if is_ready:   # 다음 목표는 잰 자세에서 따라간다 (접은 자세로 가지 않는다)
			assert steps[-1][3] in (STATE_RAMPING, STATE_FOLLOWING)
	# rested 뒤 토크 끔
	logic = rest_logic()
	assert logic.rest(0.0) is None
	run_logic(logic, 0.0, 100)
	assert logic.controller.state == STATE_RESTED
	logic.on_arm_ready(False)
	assert logic.controller.state == STATE_IDLE
	# 이상 비트: 그 주기에 잰 자세를 한 번 보내고 error. 풀려도 접은 자세로 돌아가지 않는다
	logic = rest_logic()
	assert logic.rest(0.0) is None
	steps = run_logic(logic, 0.0, 30)
	measured = steps[-1][1]
	logic.on_motor_states(1.0, arm_states(measured, arm_elbow_flex={'status': 32}))
	assert logic.step(1.0) == measured
	assert logic.controller.state == STATE_ERROR
	steps = run_logic(logic, 2.0, 5, ticks=measured)
	assert all(state not in (STATE_RESTING, STATE_RESTED) for *_, state in steps)
	assert all(command is None for _, _, command, _ in steps)



def test_tick_clamp_follows_the_last_command_while_following_and_the_measured_pose_otherwise() -> None:
	logic = ready_logic()
	home_goal(logic, 0.0)
	assert logic.step(0.0) == pytest.approx(HOME_TICKS, abs=1)
	follow_step_ticks = round(FOLLOW_STEP * MAX_TICK / 360.0)   # 512
	# 팔이 안 움직여도 (잰 tick 그대로) 명령 tick은 지난 명령에서 512 tick씩 목표로 간다
	far = HOME_TICKS['arm_shoulder_pan'] + 1000
	sent = []
	for cycle in range(1, 4):
		now = cycle * PERIOD
		logic.on_motor_states(now, arm_states(HOME_TICKS))
		home_goal(logic, now, arm_shoulder_pan=far)
		sent.append(logic.step(now)['arm_shoulder_pan'])
	start = HOME_TICKS['arm_shoulder_pan']
	assert sent[0] - start <= follow_step_ticks and sent[1] - sent[0] <= follow_step_ticks
	assert sent[-1] == pytest.approx(far, abs=1)
	# 멈춘 뒤 (arm_ready 다시) 첫 명령은 다시 잰 tick ± 114
	logic.on_arm_ready(True)
	logic.on_motor_states(1.0, arm_states(HOME_TICKS))
	home_goal(logic, 1.0, arm_shoulder_pan=HOME_TICKS['arm_shoulder_pan'] + 60)
	ticks = logic.step(1.0)
	assert abs(ticks['arm_shoulder_pan'] - HOME_TICKS['arm_shoulder_pan']) <= STEP_TICKS


def test_follow_step_must_be_positive() -> None:
	with pytest.raises(ValueError):
		make_controller(follow_step_deg=0.0)
