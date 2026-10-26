"""protocol 시험. ROS · zmq 없이 돈다."""

import base64
import json
import struct

import pytest

from lekiwi_zmq_host.protocol import (
	ARM_JOINTS,
	ARM_KEYS,
	STATE_NAMES,
	build_observation,
	compressed_image_data,
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


@pytest.mark.parametrize('text', [
	'{"x.vel": 1' + '0' * 400 + '}',   # float 로 바꾸면 OverflowError
	'{"x.vel": 1e306}',                # 바퀴 변환에서 inf → NaN
	'{"arm_gripper.pos": -1e5, "x.vel": 0}',
])
def test_huge_value_drops_the_whole_command(text):
	cmd = parse_command(text)
	assert cmd.base is None and cmd.problems


@pytest.mark.parametrize('text', ['not json', '[1, 2]', '"x"', '', '[' * 100000])
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


def _cdr_compressed_image(data: bytes, frame_id: str = 'front_camera_optical_frame', fmt: str = 'jpeg',
		little: bool = True) -> bytes:
	"""sensor_msgs/CompressedImage 를 CDR 로 손으로 직렬화한다 (머리말 4 바이트 뒤 기준 4 바이트 정렬)."""
	e = '<' if little else '>'
	body = bytearray(struct.pack(f'{e}iI', 12, 34))

	def put_string(text: str) -> None:
		while len(body) % 4:
			body.append(0)
		raw = text.encode() + b'\x00'
		body.extend(struct.pack(f'{e}I', len(raw)) + raw)

	put_string(frame_id)
	put_string(fmt)
	while len(body) % 4:
		body.append(0)
	body.extend(struct.pack(f'{e}I', len(data)) + data)
	return (b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00') + bytes(body)


@pytest.mark.parametrize('little', [True, False])
@pytest.mark.parametrize('frame_id', ['', 'a', 'ab', 'abc', 'front_camera_optical_frame'])
def test_compressed_image_data_reads_jpeg_bytes(little, frame_id):
	"""문자열 길이에 따라 정렬 빈칸이 달라져도 data 를 정확히 꺼낸다."""
	jpeg = bytes(range(256)) * 3
	assert compressed_image_data(_cdr_compressed_image(jpeg, frame_id=frame_id, little=little)) == jpeg


def test_compressed_image_data_rejects_bad_shapes():
	good = _cdr_compressed_image(b'\xff\xd8\xff\xd9')
	assert compressed_image_data(good[:-1]) is None        # data 가 잘림
	assert compressed_image_data(good[:10]) is None        # 문자열 길이가 잘림
	assert compressed_image_data(b'\x01\x02' + good[2:]) is None   # 모르는 머리말
	assert compressed_image_data(b'') is None
	huge = bytearray(good)
	huge[-8:-4] = struct.pack('<I', 1 << 30)               # data 길이가 실제보다 크다
	assert compressed_image_data(bytes(huge)) is None
