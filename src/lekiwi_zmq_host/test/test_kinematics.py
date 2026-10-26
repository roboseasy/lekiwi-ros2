"""kinematics 시험. ROS 없이 돈다. lerobot 원본과 대조하는 시험은 lerobot 이 있을 때만."""

import math
import random
from pathlib import Path

import pytest
import yaml

from lekiwi_zmq_host.kinematics import (
	BaseKinematics,
	lerobot_action_to_cmd_vel,
	lerobot_body_to_wheel_raw,
	wheel_raw_to_radps,
)

SOURCE = Path(__file__).resolve().parents[2]   # 워크스페이스 src
TOLERANCE = 1.0e-9


def robot_base() -> BaseKinematics:
	"""bringup 이 베이스 컨트롤러에 넣는 값: 패키지 설정 → base_frame.yaml → 장비 프로파일 (뒤가 이긴다)."""
	values = {}
	for path in (
		SOURCE / 'lekiwi_motor' / 'lekiwi_base_controller' / 'config' / 'lekiwi_base_controller.yaml',
		SOURCE / 'lekiwi_description' / 'config' / 'base_frame.yaml',
		SOURCE / 'lekiwi_bringup' / 'config' / 'robot_profile.yaml',
	):
		section = yaml.safe_load(path.read_text()).get('lekiwi_base_controller_node', {})
		values.update(section.get('ros__parameters', {}))
	return BaseKinematics(
		values['wheel_radius'], values['base_radius'], values['kinematics_frame_x'],
		values['kinematics_frame_y'], values['kinematics_frame_yaw'],
		values['wheel_ids'], values['wheel_directions'])


def omni(wheel_radius=0.05, base_radius=0.13647, x=0.0, y=0.0, yaw=0.0, ids=(7, 8, 9)):
	"""C++ OmniKinematics(wheel_radius, base_radius, …, x, y, yaw) 와 같은 것. ids 는 모델 바퀴 left · back · right."""
	return BaseKinematics(wheel_radius, base_radius, x, y, yaw, ids, (1.0, 1.0, 1.0))


def slots(base: BaseKinematics, speeds: dict[int, float], ids=(7, 8, 9)) -> tuple[float, ...]:
	return tuple(speeds[motor_id] for motor_id in ids)


def test_cmd_vel_reproduces_lerobot_wheel_speeds_on_the_robot_base():
	base = robot_base()
	rng = random.Random(20261026)
	for _ in range(1000):
		x, y, theta = rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-90.0, 90.0)
		expected = {
			motor_id: wheel_raw_to_radps(raw)
			for motor_id, raw in lerobot_body_to_wheel_raw(x, y, theta).items()}
		actual = base.wheel_speeds(*lerobot_action_to_cmd_vel(x, y, theta, base))
		assert set(actual) == {7, 8, 9}
		for motor_id, speed in expected.items():
			assert actual[motor_id] == pytest.approx(speed, abs=TOLERANCE)


def test_body_from_wheels_inverts_wheel_speeds_with_directions():
	base = BaseKinematics(0.05, 0.13647, 0.0076, -0.0108, 2.0944, (8, 9, 7), (1.0, -1.0, 1.0))
	for twist in ((0.2, -0.1, 0.4), (0.0, 0.0, -1.2), (-0.15, 0.2, 0.0)):
		assert base.body_from_wheels(base.wheel_speeds(*twist)) == pytest.approx(twist, abs=TOLERANCE)


def test_matches_lerobot_body_to_wheel_raw_and_raw_to_degps():
	lekiwi = pytest.importorskip('lerobot.robots.lekiwi.lekiwi')
	robot = lekiwi.LeKiwi   # 쓰는 메서드는 self 의 상태를 보지 않는다 (staticmethod _degps_to_raw 만)
	rng = random.Random(7)
	for index in range(1000):
		# 절반은 max_raw 3000 을 넘겨 비례 축소까지 본다
		limit = 0.3 if index % 2 else 1.5
		x, y, theta = rng.uniform(-limit, limit), rng.uniform(-limit, limit), rng.uniform(-300.0, 300.0)
		original = robot._body_to_wheel_raw(robot, x, y, theta)
		ours = lerobot_body_to_wheel_raw(x, y, theta)
		assert ours == {
			7: original['base_left_wheel'], 8: original['base_back_wheel'], 9: original['base_right_wheel']}
	for raw in (-3000, -1, 0, 1, 1234, 32767):
		assert wheel_raw_to_radps(raw) == pytest.approx(
			math.radians(robot._raw_to_degps(raw)), abs=1e-12)


def test_measured_check_values_pure_rotation_and_translation():
	base = robot_base()
	vx, vy, wz = lerobot_action_to_cmd_vel(0.0, 0.0, 90.0, base)
	assert wz == pytest.approx(0.916 * math.pi / 2.0, rel=0.01)
	assert (vx, vy) == pytest.approx((-0.0155, -0.0109), rel=0.01)
	assert lerobot_action_to_cmd_vel(0.1, 0.0, 0.0, base) == pytest.approx((0.1, 0.0, 0.0), abs=1e-4)
	assert lerobot_action_to_cmd_vel(0.0, 0.1, 0.0, base) == pytest.approx((0.0, 0.1, 0.0), abs=1e-4)


# ── C++ test/test_omni_kinematics.cpp 의 기대값 (바퀴 상한 안의 입력만: 상한 · 축소는 옮기지 않았다) ──

def test_cpp_forward_lateral_and_yaw_sign_patterns():
	base = omni()
	left, back, right = slots(base, base.wheel_speeds(0.1, 0.0, 0.0))
	assert left < 0.0 and back == pytest.approx(0.0, abs=TOLERANCE) and right > 0.0
	left, back, right = slots(base, base.wheel_speeds(0.0, 0.1, 0.0))
	assert left > 0.0 and back < 0.0 and right > 0.0
	assert all(speed > 0.0 for speed in slots(base, base.wheel_speeds(0.0, 0.0, 0.1)))


def test_cpp_inverse_restores_nominal_body_twist():
	base = omni()
	assert base.body_from_wheels(base.wheel_speeds(0.2, -0.1, 0.4)) == pytest.approx(
		(0.2, -0.1, 0.4), abs=TOLERANCE)


def test_cpp_ninety_degree_base_rotation():
	base = omni(0.05, 0.125, 0.0, 0.0, math.acos(-1.0) / 2.0)
	speeds = base.wheel_speeds(0.1, 0.0, 0.0)
	assert slots(base, speeds) == pytest.approx((-1.0, 2.0, -1.0), abs=TOLERANCE)
	assert base.body_from_wheels(speeds) == pytest.approx((0.1, 0.0, 0.0), abs=TOLERANCE)


def test_cpp_classroom_forward_uses_new_front_direction():
	base = omni(0.0508, 0.115, 0.007587843843, -0.010776931094, 2.094379865161)
	left, back, right = slots(base, base.wheel_speeds(0.1, 0.0, 0.0))
	assert left == pytest.approx(0.0, abs=4.0e-5) and back > 0.0 and right < 0.0


def test_cpp_pure_motor_origin_rotation_lever_arm():
	base = omni(0.05, 0.125, 0.1, -0.2, 0.7)
	assert base.body_from_wheels({7: 2.5, 8: 2.5, 9: 2.5}) == pytest.approx(
		(-0.2, -0.1, 1.0), abs=TOLERANCE)


def test_cpp_physical_wheel_ids_with_default_parameters():
	# C++ BaseControllerParameters 기본값 (wheel_ids 8 · 9 · 7)
	base = omni(0.05, 0.13647, 0.007587843843, -0.010776931094, 2.094379865161, ids=(8, 9, 7))
	for vx in (-0.03, 0.03):
		assert base.wheel_speeds(vx, 0.0, 0.0)[8] == pytest.approx(0.0, abs=1.0e-5)
	forward = base.wheel_speeds(0.03, 0.0, 0.0)
	assert forward[7] < -0.1 and forward[9] > 0.1
	lateral = base.wheel_speeds(0.0, 0.03, 0.0)
	assert lateral[7] > 0.1 and lateral[8] < -0.1 and lateral[9] > 0.1
	assert all(speed > 0.1 for speed in base.wheel_speeds(0.0, 0.0, 0.1).values())


@pytest.mark.parametrize('arguments', [
	(0.0, 0.125, 0.0, 0.0, 0.0, (7, 8, 9), (1.0, 1.0, 1.0)),
	(0.05, 0.125, float('nan'), 0.0, 0.0, (7, 8, 9), (1.0, 1.0, 1.0)),
	(0.05, 0.125, 0.0, 0.0, 0.0, (1, 2, 3), (1.0, 1.0, 1.0)),
	(0.05, 0.125, 0.0, 0.0, 0.0, (7, 8, 9), (1.0, 0.0, 1.0)),
])
def test_rejects_parameters_the_base_controller_would_not_take(arguments):
	with pytest.raises(ValueError):
		BaseKinematics(*arguments)
