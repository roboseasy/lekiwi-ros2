"""lerobot LeKiwi 호스트 ZMQ 메시지: 명령 해석 · 관측 조립 (ROS · zmq 없이).

명령 (PC → 5555): lerobot action dict 의 JSON. 관측 (5556 → PC): lerobot LeKiwi 관측과 같은 키의 JSON 하나 +
"_lekiwi_ros" (바닐라 LeKiwiClient 는 모르는 키를 무시한다).
"""

import base64
import json
import math
from dataclasses import dataclass, field
from typing import Any, Final

# lerobot LeKiwi observation.state · action 키 (LeKiwiClient._state_ft 순서)
ARM_JOINTS: Final[tuple[str, ...]] = (
	'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
	'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper')
ARM_KEYS: Final[tuple[str, ...]] = tuple(f'{name}.pos' for name in ARM_JOINTS)
BASE_KEYS: Final[tuple[str, ...]] = ('x.vel', 'y.vel', 'theta.vel')
STATE_NAMES: Final[tuple[str, ...]] = ARM_KEYS + BASE_KEYS
CAMERA_NAMES: Final[tuple[str, ...]] = ('front', 'wrist')

# [s] 이보다 오래된 상태면 관측을 보내지 않는다 (LeKiwiClient 가 빠진 키를 0.0 으로 채우므로)
STATE_TIMEOUT_S: Final[float] = 0.5
# [s] 이보다 오래된 카메라 프레임은 싣지 않는다
CAMERA_TIMEOUT_S: Final[float] = 1.0
PROTOCOL_VERSION: Final[int] = 1
# 명령 값의 절댓값 상한: 더 크면 바퀴 변환이 inf → NaN 이 된다 (lerobot 은 큰 속도를 최대 속도로 줄이므로 쓰는 값은 훨씬 작다)
MAX_COMMAND_ABS: Final[float] = 1e4


@dataclass
class Command:
	"""해석한 명령.

	Attributes:
		arm: {관절 이름 (.pos 없음): lerobot 단위} 6개, 팔 명령이 없거나 버렸으면 None.
		base: lerobot 바퀴 명령 (x m/s, y m/s, theta deg/s). 명령 전체를 버렸으면 None.
		problems: 버린 이유 (한국어).
	"""

	arm: dict[str, float] | None
	base: tuple[float, float, float] | None
	problems: list[str] = field(default_factory=list)


def _is_number(value: Any) -> bool:
	if not isinstance(value, (int, float)) or isinstance(value, bool):
		return False
	try:
		return abs(float(value)) <= MAX_COMMAND_ABS   # NaN · inf 는 False
	except OverflowError:   # 아주 큰 JSON 정수
		return False


def parse_command(text: str) -> Command:
	"""명령 JSON → Command.

	팔 키 6개는 모두 있거나 모두 없어야 한다 (일부면 팔 부분만 버림). 바퀴 키는 없으면 0.
	아는 키의 값이 유한한 수가 아니거나 MAX_COMMAND_ABS 보다 크면 명령 전체를 버린다. 모르는 키는 무시한다.
	"""
	try:
		data = json.loads(text)
	except (TypeError, ValueError, RecursionError):   # RecursionError: 아주 깊이 겹친 JSON
		data = None
	if not isinstance(data, dict):
		return Command(None, None, ['명령을 읽을 수 없습니다 (JSON 객체가 아님)'])
	bad = [key for key in STATE_NAMES if key in data and not _is_number(data[key])]
	if bad:
		return Command(None, None, [f'값이 유한한 수가 아니거나 너무 큰 명령을 버립니다: {", ".join(bad)}'])
	problems = []
	present = [key for key in ARM_KEYS if key in data]
	arm = None
	if len(present) == len(ARM_KEYS):
		arm = {name: float(data[key]) for name, key in zip(ARM_JOINTS, ARM_KEYS)}
	elif present:
		problems.append(f'팔 관절 6개가 모두 있어야 합니다 (받은 것 {len(present)}개) — 팔 명령을 버립니다')
	base = tuple(float(data.get(key, 0.0)) for key in BASE_KEYS)
	return Command(arm, base, problems)


def state_problem(names: list[str], values: list[float]) -> str | None:
	"""lekiwi/lerobot/state 를 관측에 쓸 수 없는 이유. 쓸 수 있으면 None."""
	if tuple(names) != STATE_NAMES or len(values) != len(names):
		return f'lekiwi/lerobot/state 의 이름 · 개수가 lerobot LeKiwi 와 다릅니다: {list(names)}'
	if not all(math.isfinite(value) for value in values):
		return 'lekiwi/lerobot/state 에 유한하지 않은 값이 있습니다'
	return None


def _age_ms(now: float, stamp: float) -> int:
	return round((now - stamp) * 1000.0)


def build_observation(
	state: dict[str, Any] | None,
	cameras: dict[str, tuple[bytes, float]],
	meta: dict[str, Any],
	now: float,
) -> str | None:
	"""관측 JSON. 상태가 없거나 STATE_TIMEOUT_S 보다 오래됐으면 None (보내지 않는다).

	Args:
		state: {'values': {이름: 값} (STATE_NAMES 9개), 'time': 받은 시각 [s]} 또는 None.
		cameras: {카메라 이름: (JPEG 바이트, 받은 시각 [s])}. 다시 압축하지 않고 base64 로만 싣는다.
		meta: "_lekiwi_ros" 에 더할 값 (session · reason · follower_state).
		now: 지금 시각 [s] (state · cameras 와 같은 시계).
	"""
	if not state or now - state['time'] > STATE_TIMEOUT_S:
		return None
	observation: dict[str, Any] = {name: state['values'][name] for name in STATE_NAMES}
	ages: dict[str, int | None] = {}
	for name in CAMERA_NAMES:
		frame = cameras.get(name)
		if frame is None or now - frame[1] > CAMERA_TIMEOUT_S:
			ages[f'{name}_age_ms'] = None
			continue
		observation[name] = base64.b64encode(frame[0]).decode('ascii')
		ages[f'{name}_age_ms'] = _age_ms(now, frame[1])
	observation['_lekiwi_ros'] = {
		'v': PROTOCOL_VERSION, 'jpeg': 'standard', 'units': 'degrees',
		**meta,
		'state_age_ms': _age_ms(now, state['time']),
		**ages,
	}
	return json.dumps(observation, ensure_ascii=False)
