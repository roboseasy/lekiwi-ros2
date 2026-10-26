"""protocol 시험. ROS · zmq 없이 돈다."""

import base64
import json

import pytest

from lekiwi_zmq_host.protocol import (
	ARM_JOINTS,
	ARM_KEYS,
	STATE_NAMES,
	build_observation,
	parse_command,
	state_problem,
)

ARM = {key: float(index) for index, key in enumerate(ARM_KEYS)}


def command(**values) -> str:
	return json.dumps(values)


def test_full_command():
	cmd = parse_command(json.dumps({**ARM, 'x.vel': 0.1, 'y.vel': -0.2, 'theta.vel': 30}))
	assert cmd.arm == {name: float(index) for index, name in enumerate(ARM_JOINTS)}
	assert cmd.base == (0.1, -0.2, 30.0)
	assert cmd.problems == []


def test_partial_arm_drops_only_the_arm():
	cmd = parse_command(json.dumps({'arm_gripper.pos': 50.0, 'x.vel': 0.1}))
	assert cmd.arm is None
	assert cmd.base == (0.1, 0.0, 0.0)
	assert len(cmd.problems) == 1 and '6개' in cmd.problems[0]


def test_missing_wheel_keys_are_zero_and_unknown_keys_ignored():
	cmd = parse_command(json.dumps({**ARM, 'speed': 'fast', 'observation.state': [1, 2]}))
	assert cmd.base == (0.0, 0.0, 0.0)
	assert cmd.arm is not None and cmd.problems == []
	assert parse_command('{}').base == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 'fast', True, None, [1.0]])
def test_non_finite_value_drops_the_whole_command(bad):
	cmd = parse_command(json.dumps({**ARM, 'x.vel': bad}))
	assert cmd.base is None and cmd.arm is None
	assert 'x.vel' in cmd.problems[0]
	cmd = parse_command(json.dumps({**ARM, 'arm_elbow_flex.pos': bad}))
	assert cmd.base is None and cmd.arm is None


@pytest.mark.parametrize('text', ['not json', '[1, 2]', '"x"', ''])
def test_unreadable_command(text):
	cmd = parse_command(text)
	assert cmd.base is None and cmd.problems


def state(now: float, age: float = 0.0) -> dict:
	return {'values': {name: float(index) for index, name in enumerate(STATE_NAMES)}, 'time': now - age}


def test_no_state_or_stale_state_sends_nothing():
	assert build_observation(None, {}, {}, 10.0) is None
	assert build_observation({}, {}, {}, 10.0) is None
	assert build_observation(state(10.0, age=0.51), {}, {}, 10.0) is None
	assert build_observation(state(10.0, age=0.49), {}, {}, 10.0) is not None


def test_observation_keys_cameras_and_meta():
	jpeg = b'\xff\xd8 not decoded \xff\xd9'
	meta = {'session': 'idle', 'reason': None, 'follower_state': 'following'}
	text = build_observation(
		state(10.0, age=0.012), {'front': (jpeg, 9.98), 'wrist': (jpeg, 8.9)}, meta, 10.0)
	observation = json.loads(text)
	assert list(observation)[:9] == list(STATE_NAMES)
	assert [observation[name] for name in STATE_NAMES] == [float(i) for i in range(9)]
	assert base64.b64decode(observation['front']) == jpeg
	assert 'wrist' not in observation   # 1 s 넘음
	assert observation['_lekiwi_ros'] == {
		'v': 1, 'jpeg': 'standard', 'units': 'degrees', **meta,
		'state_age_ms': 12, 'front_age_ms': 20, 'wrist_age_ms': None}


def test_state_problem():
	values = [0.0] * 9
	assert state_problem(list(STATE_NAMES), values) is None
	assert state_problem(list(reversed(STATE_NAMES)), values) is not None
	assert state_problem(list(STATE_NAMES)[:8], values[:8]) is not None
	assert state_problem(list(STATE_NAMES), values[:8]) is not None
	assert state_problem(list(STATE_NAMES), [float('nan')] + values[1:]) is not None
