"""팔로워 팔 계산: tick ↔ LeRobot 단위 변환, 입력 검사, 시작 램프 · 이동 제한 · 끊김 유지,
lerobot LeKiwi 관측 상태의 몸체 속도 (바퀴 → x.vel · y.vel · theta.vel).

ROS를 import하지 않는다. lerobot은 ArmConverter를 만들 때만 불러오므로
FollowerController · FollowerArmLogic · base_velocity 시험은 lerobot 없이도 돈다.

LeRobot 단위 (lerobot LeKiwi, use_degrees=True)
	몸 관절 5개   각도 [deg]. 보정 범위(range_min~range_max) 가운데가 0°
	그리퍼        0~100. range_min이 0, range_max가 100
	몸체 속도     x.vel · y.vel [m/s], theta.vel [deg/s]
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
from typing import Any, Final

import numpy as np

# lerobot LeKiwi(lerobot/robots/lekiwi/lekiwi.py)와 같은 이름 · 모터 ID
ARM_JOINTS: Final[tuple[str, ...]] = (
	'arm_shoulder_pan',
	'arm_shoulder_lift',
	'arm_elbow_flex',
	'arm_wrist_flex',
	'arm_wrist_roll',
	'arm_gripper',
)
ARM_IDS: Final[dict[str, int]] = {
	name: index + 1 for index, name in enumerate(ARM_JOINTS)}
GRIPPER: Final[str] = 'arm_gripper'
MOTOR_MODEL: Final[str] = 'sts3215'
MAX_TICK: Final[int] = 4095   # lerobot DEGREES 계산의 분모 (해상도 4096 - 1)
# _normalize · _unnormalize(내부 함수)를 0.6.0에서 확인했다. 다른 판은 시험부터 다시 본다
SUPPORTED_LEROBOT: Final[str] = '0.6'
# lerobot LeKiwi.configure가 팔 모터에 쓰는 (P, I, D)
LEROBOT_PID: Final[tuple[int, int, int]] = (16, 0, 32)
# MotorState.status 이상 비트 (lekiwi_msgs/MotorState STATUS_*) → 이름
FAULT_BITS: Final[dict[int, str]] = {
	1: '전압', 2: '각도 센서', 4: '과열', 8: '과전류', 32: '과부하'}
FAULT_MASK: Final[int] = sum(FAULT_BITS)
OVERLOAD_BIT: Final[int] = 32

# lerobot LeKiwi 바퀴 이름 → 모터 ID (lekiwi.py). 실물도 왼쪽 7 · 뒤 8 · 오른쪽 9다
# (lekiwi_description/docs/FRAME_MIGRATION.md)
BASE_WHEELS: Final[dict[str, int]] = {
	'base_left_wheel': 7, 'base_back_wheel': 8, 'base_right_wheel': 9}
# lerobot LeKiwi._wheel_raw_to_body 기본값. ROS odom(베이스 컨트롤러)은 몸체 반지름 0.13647 m를 쓴다
LEROBOT_WHEEL_RADIUS: Final[float] = 0.05   # [m]
LEROBOT_BASE_RADIUS: Final[float] = 0.125   # [m]
# 드라이버 MotorState.velocity [rad/s] = Present_Velocity [tick/s] × 2π / 4096 (sts3215 TicksToRad)
TICKS_PER_RAD: Final[float] = 4096 / (2 * math.pi)
# lerobot LeKiwi 데이터셋 observation.state · action 이름 · 순서 (LeKiwi._state_ft)
LEROBOT_STATE_NAMES: Final[tuple[str, ...]] = (
	*(f'{name}.pos' for name in ARM_JOINTS), 'x.vel', 'y.vel', 'theta.vel')

# lekiwi/arm/status의 state 값
STATE_IDLE: Final[str] = 'idle'
STATE_RAMPING: Final[str] = 'ramping'
STATE_FOLLOWING: Final[str] = 'following'
STATE_ERROR: Final[str] = 'error'
STATE_RESTING: Final[str] = 'resting'   # lekiwi/arm/rest: 접은 자세로 돌아가는 중
STATE_RESTED: Final[str] = 'rested'     # 접은 자세에 왔다 (팔 토크를 꺼도 된다)
# 접은 자세 램프가 끝난 뒤 이만큼 [s] 기다려도 다 오지 않으면 경고와 함께 rested로 본다
REST_SETTLE_S: Final[float] = 1.0

Pose = dict[str, float]   # 관절 이름 → LeRobot 단위 값


def lerobot_version_error(version: str) -> str | None:
	"""쓸 수 없는 lerobot 판이면 이유, 0.6.x면 None.

	Args:
		version: lerobot.__version__.
	"""
	if version.split('.')[:2] == SUPPORTED_LEROBOT.split('.'):
		return None
	return (
		f'lerobot {version}은 쓸 수 없습니다 ({SUPPORTED_LEROBOT}.x만: '
		'단위 변환이 lerobot 내부 함수 _normalize · _unnormalize를 씁니다)')


def fault_mask(name: str) -> int:
	"""관절이 명령을 막는 이상 비트. 그리퍼는 과부하를 뺀다."""
	return FAULT_MASK & ~OVERLOAD_BIT if name == GRIPPER else FAULT_MASK


def arm_motor_error(motors: Iterable[Any]) -> str | None:
	"""모터 상태에서 명령을 막을 팔 모터 이상: 이름과 ID가 다르거나 이상 비트가 섰다.

	그리퍼 과부하는 넣지 않는다: 물건을 쥐면 생기고, 모터가 스스로 힘을 줄인다
	(FollowerArmLogic이 경고로 알린다).

	Args:
		motors: lekiwi_msgs/MotorState 목록 (name · id · online · status).

	Returns:
		사람이 읽는 오류 (한국어). 없으면 None.
	"""
	arm = {motor.name: motor for motor in motors if motor.name in ARM_IDS}
	wrong_ids = [
		f'{name} {motor.id} (LeKiwi {ARM_IDS[name]})'
		for name, motor in arm.items() if motor.id != ARM_IDS[name]]
	if wrong_ids:
		return f'팔 모터 ID가 다릅니다: {", ".join(wrong_ids)}'
	faults = [
		f'{name} ' + ' · '.join(
			label for bit, label in FAULT_BITS.items()
			if motor.status & fault_mask(name) & bit)
		for name, motor in arm.items()
		if motor.online and motor.status & fault_mask(name)]
	if faults:
		return f'팔 모터 이상으로 명령을 멈춥니다: {", ".join(faults)}'
	return None


def compare_registers(
	registers: Any,
	calibration: Mapping[str, tuple[int, int, int]],
	robot_id: str,
) -> tuple[str | None, str | None]:
	"""드라이버가 읽은 팔 모터 레지스터를 보정 파일과 맞춘다 (lerobot is_calibrated 기준).

	Args:
		registers: lekiwi_msgs/ArmRegisters (ids · valid · homing_offset ·
			min_position_limit · max_position_limit · p/i/d_coefficient).
		calibration: 관절 이름 → 보정 파일의 (homing_offset, range_min, range_max).
		robot_id: 보정 파일 이름 (오류 글에 쓴다).

	Returns:
		(오류, 경고). 오류가 있으면 명령하지 않는다: 값이 다르거나 드라이버가
		읽지 못한 관절이 있다. 경고는 P · I · D가 lerobot LeKiwi와 다를 때다.
	"""
	prefix = f'보정 파일이 이 로봇의 모터와 다릅니다 ({robot_id}.json)'
	fields = (
		registers.valid, registers.homing_offset, registers.min_position_limit,
		registers.max_position_limit, registers.p_coefficient,
		registers.i_coefficient, registers.d_coefficient)
	if any(len(field) != len(registers.ids) for field in fields):
		return f'{prefix}: 드라이버 레지스터 메시지의 길이가 맞지 않습니다', None
	index = {motor_id: i for i, motor_id in enumerate(registers.ids)}
	problems = []
	gains = []
	for name in ARM_JOINTS:
		i = index.get(ARM_IDS[name])
		if i is None or not registers.valid[i]:
			problems.append(f'{name} 읽지 못함')
			continue
		measured = (
			registers.homing_offset[i],
			registers.min_position_limit[i],
			registers.max_position_limit[i])
		for field, got, want in zip(
			('homing_offset', 'range_min', 'range_max'),
			measured,
			calibration[name],
		):
			if got != want:
				problems.append(f'{name} {field} 모터 {got} · 파일 {want}')
		pid = (
			registers.p_coefficient[i],
			registers.i_coefficient[i],
			registers.d_coefficient[i])
		if pid != LEROBOT_PID:
			gains.append(f'{name} P {pid[0]} · I {pid[1]} · D {pid[2]}')
	error = f'{prefix}: {", ".join(problems)}' if problems else None
	warning = (
		'팔 모터 PID가 lerobot LeKiwi(P 16 · I 0 · D 32)와 다릅니다: '
		+ ', '.join(gains)) if gains else None
	return error, warning


def wheel_raw_to_body(
	left: float, back: float, right: float,
) -> tuple[float, float, float]:
	"""바퀴 Present_Velocity → 몸체 속도. lerobot 0.6.0 LeKiwi._wheel_raw_to_body와 같은 계산.

	같은 상수 (바퀴 반지름 0.05 m, 몸체 반지름 0.125 m, 바퀴 각도 [240, 0, 120] - 90°),
	같은 순서 (tick/s → deg/s → rad/s → m/s → M⁻¹), 같은 numpy 연산이다.
	lerobot을 올리면 단위 시험 (lerobot 함수와 비교)을 먼저 돌린다.

	Args:
		left, back, right: base_left_wheel · base_back_wheel · base_right_wheel의
			Present_Velocity [tick/s] (부호-크기를 푼 값).

	Returns:
		(x.vel [m/s], y.vel [m/s], theta.vel [deg/s]).
	"""
	steps_per_deg = 4096.0 / 360.0
	wheel_degps = np.array(
		[left / steps_per_deg, back / steps_per_deg, right / steps_per_deg])
	wheel_linear_speeds = wheel_degps * (np.pi / 180.0) * LEROBOT_WHEEL_RADIUS
	angles = np.radians(np.array([240, 0, 120]) - 90)
	m = np.array([[np.cos(a), np.sin(a), LEROBOT_BASE_RADIUS] for a in angles])
	x, y, theta_rad = np.linalg.inv(m).dot(wheel_linear_speeds)
	return float(x), float(y), float(theta_rad * (180.0 / np.pi))


def base_velocity(motors: Iterable[Any]) -> tuple[float, float, float] | None:
	"""모터 상태의 바퀴 3개 → lerobot LeKiwi 몸체 속도.

	lerobot LeKiwi.get_observation처럼 바퀴 Present_Velocity로 wheel_raw_to_body를 계산한다
	(ROS odom을 쓰지 않는다: 몸체 반지름 · 단위 · 0.3 s 창이 다르다).

	Args:
		motors: lekiwi_msgs/MotorState 목록 (name · id · online · velocity [rad/s]).

	Returns:
		(x.vel [m/s], y.vel [m/s], theta.vel [deg/s]). 바퀴 하나라도 없거나, 응답하지 않았거나,
		ID가 lerobot LeKiwi와 다르거나, 속도가 유한하지 않으면 None.
	"""
	wheels = {motor.name: motor for motor in motors}
	if not all(
		name in wheels and wheels[name].online and wheels[name].id == motor_id
		and math.isfinite(wheels[name].velocity)
		for name, motor_id in BASE_WHEELS.items()
	):
		return None
	# 드라이버 velocity는 tick × 2π / 4096이다: lerobot이 받는 정수 tick으로 되돌린다
	return wheel_raw_to_body(*(
		round(wheels[name].velocity * TICKS_PER_RAD) for name in BASE_WHEELS))


def limit_ticks(
	ticks: Mapping[str, int],
	measured: Mapping[str, int],
	max_step: int,
) -> dict[str, int]:
	"""tick 명령을 잰 tick ± max_step으로 자른다.

	단위 계산 뒤의 마지막 안전망이다. lerobot은 보정 범위 밖 그리퍼를 0~100으로
	잘라 읽으므로, 범위 밖에 있는 그리퍼도 여기서 한 번에 튀지 않는다.
	"""
	return {
		name: min(
			measured[name] + max_step, max(measured[name] - max_step, tick))
		for name, tick in ticks.items()}


def make_status(
	state: str,
	reason: str,
	robot_id: str,
	diffs: Pose | None = None,
	ramp_progress: float | None = None,
	warnings: Sequence[str] = (),
) -> dict[str, Any]:
	"""lekiwi/arm/status JSON 내용 (README)."""
	return {
		'state': state,
		'reason': reason,
		'robot_id': robot_id,
		'diffs': {name: round(diff, 2) for name, diff in (diffs or {}).items()},
		'ramp_progress': (
			None if ramp_progress is None else round(ramp_progress, 3)),
		'warnings': list(warnings),
	}


class ArmConverter:
	"""LeRobot 보정 파일로 팔 tick ↔ LeRobot 단위를 바꾼다.

	포트를 열지 않은 lerobot FeetechMotorsBus의 _normalize · _unnormalize를
	그대로 쓴다. lerobot 0.6.0 내부 함수라서, lerobot을 올리면 단위 시험
	(공식과 비교)이 먼저 잡아야 한다.

	Attributes:
		ids: 관절 이름 → 모터 ID (1~6).
		ranges: 관절 이름 → 보정 (range_min, range_max) [tick].
		homing_offsets: 관절 이름 → 보정 homing_offset (부호 있는 tick).
		limits: 관절 이름 → 보정 범위의 LeRobot 단위 (낮은 값, 높은 값).
		units_per_degree: 관절 이름 → 모터 축 1°에 해당하는 LeRobot 단위.
			몸 관절은 1, 그리퍼는 100 / (보정 범위 각도) (lekiwi04는 약 0.70).
	"""

	def __init__(self, calibration_path: Path) -> None:
		"""보정 파일을 읽고 변환용 버스를 만든다.

		Args:
			calibration_path: lerobot 보정 JSON (~/.cache/huggingface/lerobot/
				calibration/robots/lekiwi/<robot_id>.json).

		Raises:
			OSError: 파일을 읽지 못했다.
			ValueError: 팔 관절이 빠졌거나 모터 ID가 lerobot LeKiwi와 다르다.
			ImportError: lerobot(feetech 포함)이 없다.
			RuntimeError: lerobot이 0.6.x가 아니다.
		"""
		# lerobot import는 Pi에서 약 5.5초 걸린다. 노드가 시작할 때 한 번만 부른다
		import draccus
		import lerobot
		from lerobot.motors import Motor, MotorCalibration, MotorNormMode
		from lerobot.motors.feetech import FeetechMotorsBus

		version_error = lerobot_version_error(lerobot.__version__)
		if version_error is not None:
			raise RuntimeError(version_error)

		# lerobot Robot._load_calibration과 같은 방법으로 읽는다
		with (
			open(calibration_path, encoding='utf-8') as file,
			draccus.config_type('json'),
		):
			calibration = draccus.load(dict[str, MotorCalibration], file)
		missing = [name for name in ARM_JOINTS if name not in calibration]
		if missing:
			raise ValueError(f'보정 파일에 팔 관절이 없습니다: {", ".join(missing)}')
		wrong_ids = [
			name for name in ARM_JOINTS
			if calibration[name].id != ARM_IDS[name]]
		if wrong_ids:
			raise ValueError(
				f'보정 파일의 모터 ID가 LeKiwi(1~6)와 다릅니다: {", ".join(wrong_ids)}')

		motors = {
			name: Motor(
				ARM_IDS[name],
				MOTOR_MODEL,
				MotorNormMode.RANGE_0_100 if name == GRIPPER
				else MotorNormMode.DEGREES,
			)
			for name in ARM_JOINTS
		}
		self._bus = FeetechMotorsBus(
			port='',
			motors=motors,
			calibration={name: calibration[name] for name in ARM_JOINTS},
		)
		# lerobot DEGREES 계산이 쓰는 값 (4096 - 1)
		self._max_tick = self._bus.model_resolution_table[MOTOR_MODEL] - 1
		self.ids: dict[str, int] = dict(ARM_IDS)
		self.ranges: dict[str, tuple[int, int]] = {
			name: (calibration[name].range_min, calibration[name].range_max)
			for name in ARM_JOINTS
		}
		self.homing_offsets: dict[str, int] = {
			name: calibration[name].homing_offset for name in ARM_JOINTS}
		self.limits: dict[str, tuple[float, float]] = {}
		self.units_per_degree: dict[str, float] = {}
		for name, (range_min, range_max) in self.ranges.items():
			ends = sorted((
				self.to_units({name: range_min})[name],
				self.to_units({name: range_max})[name],
			))
			self.limits[name] = (ends[0], ends[1])
			range_degrees = (range_max - range_min) * 360.0 / self._max_tick
			self.units_per_degree[name] = (ends[1] - ends[0]) / range_degrees

	def to_units(self, ticks: dict[str, int]) -> Pose:
		"""tick → LeRobot 단위 (lerobot _normalize).

		Args:
			ticks: 관절 이름 → Present_Position [tick].

		Returns:
			관절 이름 → LeRobot 단위. 몸 관절은 범위 밖 값도 그대로 바꾸고,
			그리퍼는 lerobot처럼 0~100으로 자른다.
		"""
		values = self._bus._normalize(
			{self.ids[name]: tick for name, tick in ticks.items()})
		return {name: values[self.ids[name]] for name in ticks}

	def to_ticks(self, units: Pose) -> dict[str, int]:
		"""LeRobot 단위 → tick (lerobot _unnormalize).

		lerobot은 몸 관절을 자르지 않는다 (999° → 13351 tick). 그래서 모터가
		받을 수 있는 0~4095로 자른다. 목표를 보정 범위로 자르는 일은
		FollowerController가 한다.

		Args:
			units: 관절 이름 → LeRobot 단위.

		Returns:
			관절 이름 → Goal_Position [tick] (0~4095).
		"""
		values = self._bus._unnormalize(
			{self.ids[name]: value for name, value in units.items()})
		return {
			name: min(self._max_tick, max(0, values[self.ids[name]]))
			for name in units
		}


class FollowerController:
	"""리더 목표를 팔로워 명령으로 바꾼다 (LeRobot 단위, 주기마다 update).

	- 목표는 보정 범위(limits)로 자른다.
	- 멈춰 있다가 목표가 오면, 팔로워와 차이가 ramp_threshold_deg를 넘는
	  관절이 있을 때 ramp_time_s 동안 팔로워 자세 → 지금 목표로 맞춘다.
	- 따라가는 중(STATE_FOLLOWING)에는 목표를 그대로 보낸다 (lerobot max_relative_target
	  None과 같다). 지난 명령에서 한 번에 follow_step_deg 넘게 튀는 값(손목 ±180° 이음매,
	  깨진 값)만 자른다. 사람이 움직이는 속도는 자르지 않는다.
	- 램프 · 접은 자세 · 멈춘 뒤 첫 명령은 **잰 자세**(모터 상태)에서 max_step_deg 넘게
	  떨어지지 않는다 (lerobot ensure_safe_goal_position과 같은 기준).
	- 몸 관절이 명령과 track_tolerance_deg 넘게 떨어진 채 track_timeout_s 동안
	  그만큼도 움직이지 않으면 (막힘 · 힘 모자람) warning에 적는다. 명령은 그대로다.
	  그리퍼는 물건을 잡으면 막히는 것이 정상이라 보지 않는다.
	- 드라이버가 팔 명령을 받지 않거나 (arm_ready false) 목표가 goal_timeout_s 넘게
	  오지 않으면 명령을 그친다 (드라이버가 마지막 목표를 유지한다).
	- 그친 뒤 다음 목표는 잰 자세에서 다시 램프 여부를 본다.
	- 접은 자세 (start_rest, lekiwi/arm/rest): 리더 목표를 무시하고 잰 자세 → 보정 범위로
	  자른 접은 자세로 rest_time_s 동안 smoothstep 램프 (STATE_RESTING). 모든 관절이
	  rest_tolerance_deg 안에 오거나, 램프가 끝나고 REST_SETTLE_S가 지나면 (못 온 관절은
	  warning) STATE_RESTED: 마지막 명령을 한 번 보내고 그친다 (드라이버가 그 목표를 유지한다).
	  halt (arm_ready 메시지 · 이상)로만 나온다.
	- 그리퍼의 각도 값(threshold · step · tolerance)은 모터 축 각도 기준으로 0~100 단위로
	  바꿔 쓴다 (units_per_degree).

	램프 명령은 늘 잰 자세와 보정 범위 사이에 있다. 그래서 팔이 범위 밖에서
	시작해도 (lekiwi04 접은 자세의 팔꿈치 105.7° > 97.2°) 한 번에 튀지 않고
	범위 안으로 들어온다.

	Attributes:
		state: STATE_IDLE · STATE_RAMPING · STATE_FOLLOWING · STATE_ERROR.
		reason: 사람이 읽는 상태 이유 (한국어). 따라가는 중에는 빈 문자열.
		diffs: 관절 이름 → (보정 범위로 자른 목표 - 팔로워) [LeRobot 단위].
			명령하는 동안만 채운다.
		ramp_progress: 램프 진행 0~1. 램프 중이 아니면 None.
		warning: 따라오지 않는 몸 관절 경고 (한국어). 없으면 None.
		max_step_deg: 램프 · 접은 자세에서 잰 자세로부터 한 번 최대 이동 [deg].
		follow_step_deg: 따라가는 중 지난 명령에서 한 번 최대 이동 [deg] (튐 막기).
		is_step_from_command: 마지막 명령을 지난 명령 기준(follow_step_deg)으로 잘랐는가.
			False면 잰 자세 기준(max_step_deg)이다.
		goal_timeout_s: 목표가 이만큼 [s] 끊기면 명령을 그친다.
	"""

	def __init__(
		self,
		limits: dict[str, tuple[float, float]],
		units_per_degree: dict[str, float],
		ramp_threshold_deg: float = 8.0,
		ramp_time_s: float = 2.0,
		max_step_deg: float = 10.0,
		goal_timeout_s: float = 0.5,
		track_tolerance_deg: float = 4.0,
		track_timeout_s: float = 0.3,
		rest_time_s: float = 3.0,
		rest_tolerance_deg: float = 3.0,
		follow_step_deg: float = 45.0,
	) -> None:
		"""설정을 받는다.

		Args:
			limits: 관절 이름 → 보정 범위 (낮은 값, 높은 값) [LeRobot 단위].
			units_per_degree: 관절 이름 → 모터 축 1°의 LeRobot 단위.
			ramp_threshold_deg: 이 차이 [deg]를 넘는 관절이 있으면 램프.
			ramp_time_s: 램프 시간 [s].
			max_step_deg: 램프 · 접은 자세 · 멈춘 뒤 첫 명령에서 잰 자세로부터 한 번
				최대 이동 [deg].
			goal_timeout_s: 목표가 이만큼 [s] 끊기면 명령을 그친다.
			track_tolerance_deg: 명령과 잰 자세가 이만큼 [deg] 넘게 차이 나면
				따라오는지 본다. max_step_deg보다 작아야 한다 (램프 중 막힌 관절 앞의
				명령은 잰 자세 + max_step_deg에 머문다).
			track_timeout_s: 그동안 [s] 따라오지 않으면 경고한다.
			rest_time_s: 접은 자세 램프 시간 [s].
			rest_tolerance_deg: 접은 자세에서 이만큼 [deg] 안이면 왔다고 본다.
			follow_step_deg: 따라가는 중 지난 명령에서 한 번 최대 이동 [deg].
				사람 손보다 크고 이음매 튐(358°)보다 작게 둔다.

		Raises:
			ValueError: 시간 · 이동 값이 0 이하이거나, threshold가 음수이거나,
				track_tolerance_deg · rest_tolerance_deg가 max_step_deg 이상이다.
		"""
		if min(
			ramp_time_s, max_step_deg, goal_timeout_s,
			track_tolerance_deg, track_timeout_s, rest_time_s, rest_tolerance_deg,
			follow_step_deg,
		) <= 0.0:
			raise ValueError('times, steps and tolerances must be positive')
		if ramp_threshold_deg < 0.0:
			raise ValueError('ramp_threshold_deg must not be negative')
		# 막힌 관절 앞에서 명령은 잰 자세 + max_step에 머문다: 그보다 작아야 잡힌다
		if track_tolerance_deg >= max_step_deg:
			raise ValueError('track_tolerance_deg must be below max_step_deg')
		# 다 왔을 때 마지막 명령이 접은 자세 그 자체가 되게 (명령은 잰 자세 + max_step까지만 간다)
		if rest_tolerance_deg >= max_step_deg:
			raise ValueError('rest_tolerance_deg must be below max_step_deg')
		self._limits = limits
		self.max_step_deg = max_step_deg
		self._thresholds = {
			name: ramp_threshold_deg * scale
			for name, scale in units_per_degree.items()}
		self._max_steps = {
			name: max_step_deg * scale
			for name, scale in units_per_degree.items()}
		self.follow_step_deg = follow_step_deg
		self._follow_steps = {
			name: follow_step_deg * scale
			for name, scale in units_per_degree.items()}
		self.is_step_from_command = False
		self._track_tolerances = {
			name: track_tolerance_deg * scale
			for name, scale in units_per_degree.items() if name != GRIPPER}
		self._rest_tolerances = {
			name: rest_tolerance_deg * scale
			for name, scale in units_per_degree.items()}
		self._ramp_time_s = ramp_time_s
		self._rest_time_s = rest_time_s
		self.goal_timeout_s = goal_timeout_s
		self._track_timeout_s = track_timeout_s
		self._rest_target: Pose | None = None   # 접은 자세 (자른 값). None이면 접은 자세 모드가 아니다
		self._rest_ramp_s = 0.0                 # 이번 접은 자세 램프 시간 (가까우면 0)
		self._command: Pose | None = None   # 마지막 명령. None이면 그친 상태
		self._ramp_start: Pose = {}
		self._ramp_start_time = 0.0
		# 관절 → (뒤처지기 시작한 시각, 그때 잰 값)
		self._lag_anchors: dict[str, tuple[float, float]] = {}
		self.state = STATE_IDLE
		self.reason = '목표를 기다립니다'
		self.diffs: Pose = {}
		self.ramp_progress: float | None = None
		self.warning: str | None = None

	def halt(self, reason: str, is_error: bool) -> None:
		"""명령을 그친다. 다음 목표는 잰 자세에서 다시 출발한다.

		Args:
			reason: 사람이 읽는 이유 (한국어).
			is_error: True면 STATE_ERROR, False면 STATE_IDLE.
		"""
		self.state = STATE_ERROR if is_error else STATE_IDLE
		self.reason = reason
		self.diffs = {}
		self.ramp_progress = None
		self.warning = None
		self._command = None
		self._rest_target = None

	def start_rest(self, now: float, follower: Pose, rest: Pose) -> None:
		"""접은 자세 모드를 시작한다: 리더 목표를 무시하고 잰 자세에서 접은 자세로 간다.

		Args:
			now: 지금 시각 [s].
			follower: 팔로워 잰 자세 (모든 관절).
			rest: 접은 자세 [LeRobot 단위]. 보정 범위로 자른다 (lekiwi04 접은
				팔꿈치 105.7°는 범위 끝 97.2°에서 멈춘다). 접은 자세가 범위 밖이고 관절이
				이미 그쪽 범위 밖에 있으면 (실물로 접혀 있다) 그 자리를 목표로 둔다:
				범위 끝까지 8° 펴지 않는다 (토크를 끄면 어차피 처진다).
		"""
		self.halt('', is_error=False)
		self._rest_target = {}
		for name, (low, high) in self._limits.items():
			value, measured = rest[name], follower[name]
			is_beyond = value > high and measured > high or value < low and measured < low
			self._rest_target[name] = measured if is_beyond else min(high, max(low, value))
		self._command = dict(follower)
		self._ramp_start = dict(follower)
		self._ramp_start_time = now
		# 이미 접은 자세면 램프 없이 바로 rested
		is_near = all(
			abs(self._rest_target[name] - follower[name]) <= tolerance
			for name, tolerance in self._rest_tolerances.items())
		self._rest_ramp_s = 0.0 if is_near else self._rest_time_s
		self.state = STATE_RESTING
		self.reason = (
			'이미 접은 자세입니다' if is_near
			else f'접은 자세로 {self._rest_ramp_s:g}초 동안 돌아갑니다')
		self.ramp_progress = 0.0

	def update(
		self,
		now: float,
		follower: Pose,
		goal: Pose | None,
		goal_time: float,
		is_arm_ready: bool,
	) -> Pose | None:
		"""한 주기 계산.

		Args:
			now: 지금 시각 [s] (단조 시계).
			follower: 팔로워 잰 자세 (모든 관절, 가장 최근 모터 상태).
			goal: 마지막으로 받은 리더 목표 (모든 관절). 없으면 None.
			goal_time: goal을 받은 시각 [s].
			is_arm_ready: 드라이버가 팔 명령을 받는가 (lekiwi/motor/arm_ready).
				false → true가 되면 잰 자세에서 새로 출발한다 (램프 여부를 다시 본다).

		Returns:
			이번 주기에 보낼 명령 (LeRobot 단위), 보내지 않을 때는 None.
		"""
		self.is_step_from_command = False
		if not is_arm_ready:
			self.halt('팔 토크가 꺼져 있습니다', is_error=False)
			return None
		if self._rest_target is not None:   # 접은 자세 모드: 리더 목표를 보지 않는다
			return self._update_rest(now, follower)
		if goal is None:
			self.halt('목표를 기다립니다', is_error=False)
			return None
		if now - goal_time > self.goal_timeout_s:
			self.halt(
				f'목표가 {self.goal_timeout_s:g}초 넘게 오지 않아 자세를 유지합니다',
				is_error=False)
			return None
		self.warning = None
		if self._command is not None:
			stuck = self._stuck_joints(now, follower)
			if stuck:
				self.warning = (
					f'팔이 명령을 따라오지 않습니다: {", ".join(stuck)} '
					'(막혔거나 힘이 모자랍니다)')

		target = {
			name: min(high, max(low, goal[name]))
			for name, (low, high) in self._limits.items()}
		self.diffs = {name: target[name] - follower[name] for name in target}
		# 이미 따라가던 중이면 지난 명령 기준으로 튐만 자른다. 출발 · 램프는 잰 자세 기준
		is_continuing = self._command is not None and self.state == STATE_FOLLOWING
		if self._command is None:
			self._start(now, follower)
		if self.state == STATE_RAMPING:
			progress = (now - self._ramp_start_time) / self._ramp_time_s
			if progress >= 1.0:
				self.state = STATE_FOLLOWING
				self.reason = ''
				self.ramp_progress = None
			else:
				self.ramp_progress = progress
				# smoothstep: 시작 · 끝 속도가 0이다
				weight = progress * progress * (3.0 - 2.0 * progress)
				target = {
					name: start + weight * (target[name] - start)
					for name, start in self._ramp_start.items()}

		if is_continuing:
			self.is_step_from_command = True
			return self._step_toward(self._command, target, self._follow_steps)
		return self._step_toward(follower, target)

	def _step_toward(self, reference: Pose, target: Pose, steps: Pose | None = None) -> Pose:
		"""reference(잰 자세 또는 지난 명령)에서 target 쪽으로 관절마다 steps(기본 max_step_deg)까지만 간 명령."""
		steps = steps or self._max_steps
		command = {}
		for name, start in reference.items():
			offset = min(steps[name], max(-steps[name], target[name] - start))
			command[name] = start + offset
		self._command = command
		return dict(command)

	def _update_rest(self, now: float, follower: Pose) -> Pose | None:
		"""접은 자세 모드의 한 주기. rested가 된 뒤에는 명령을 내지 않는다."""
		if self.state == STATE_RESTED:
			return None   # 드라이버가 마지막 목표(접은 자세)를 유지한다
		rest = self._rest_target
		self.diffs = {name: rest[name] - follower[name] for name in rest}
		elapsed = now - self._ramp_start_time
		if elapsed < self._rest_ramp_s:
			progress = elapsed / self._rest_ramp_s
			self.ramp_progress = progress
			weight = progress * progress * (3.0 - 2.0 * progress)   # smoothstep
			return self._step_toward(follower, {
				name: start + weight * (rest[name] - start)
				for name, start in self._ramp_start.items()})
		self.ramp_progress = 1.0
		late = [
			name for name, diff in self.diffs.items()
			if abs(diff) > self._rest_tolerances[name]]
		if not late or elapsed - self._rest_ramp_s >= REST_SETTLE_S:
			self.state = STATE_RESTED
			self.reason = '접은 자세입니다'
			self.ramp_progress = None
			self.warning = (
				f'접은 자세에 다 오지 못했습니다: {", ".join(late)}' if late else None)
		# 끝나는 주기에도 한 번 보낸다: 다 왔고 차이가 max_step 안이면 이 명령이 곧 접은 자세다
		return self._step_toward(follower, rest)

	def _start(self, now: float, follower: Pose) -> None:
		"""멈춰 있다가 목표가 왔다: 잰 자세에서 출발하고 램프 여부를 정한다."""
		self._command = dict(follower)
		self._lag_anchors = {}
		far = [
			name for name, diff in self.diffs.items()
			if abs(diff) > self._thresholds[name]]
		if far:
			self.state = STATE_RAMPING
			self.reason = (
				f'시작 자세 차이가 커서 {self._ramp_time_s:g}초 동안 맞춥니다: '
				f'{", ".join(far)}')
			self._ramp_start = dict(follower)
			self._ramp_start_time = now
			self.ramp_progress = 0.0
		else:
			self.state = STATE_FOLLOWING
			self.reason = ''
			self.ramp_progress = None

	def _stuck_joints(self, now: float, follower: Pose) -> list[str]:
		"""명령을 따라오지 않는 몸 관절.

		지난 명령과 tolerance 넘게 떨어진 채 track_timeout_s 동안 tolerance만큼도
		움직이지 않은 관절이다. 빨리 움직이며 뒤처지는 관절은 넣지 않는다.
		"""
		stuck = []
		for name, tolerance in self._track_tolerances.items():
			measured = follower[name]
			anchor = self._lag_anchors.get(name)
			if abs(self._command[name] - measured) <= tolerance:
				self._lag_anchors.pop(name, None)
			elif anchor is None or abs(measured - anchor[1]) > tolerance:
				self._lag_anchors[name] = (now, measured)
			elif now - anchor[0] > self._track_timeout_s:
				stuck.append(name)
		return stuck


class FollowerArmLogic:
	"""lekiwi_follower_arm_node의 판단 (ROS 없이): 콜백 입력을 모아 주기마다 보낼 tick을 정한다.

	명령하지 않는 경우 (위가 먼저):
	1. 모터 상태가 state_timeout_s 넘게 오지 않았다 (error).
	2. 팔 모터 ID가 lerobot LeKiwi와 다르거나 이상 비트(전압 · 각도 센서 · 과열 ·
	   과전류 · 과부하, 그리퍼 과부하는 경고)가 섰다 (error). 팔이 켜져 있을 때
	   걸리면 그 주기에 잰 자세를 한 번 보내 (막힌 관절이 계속 밀지 않게) 걸어
	   둔다. 이상이 풀리고 목표가 goal_timeout_s 넘게 끊기거나 (사람이 원격 조작을
	   멈췄다) arm_ready가 새로 오면 푼다. 눌린 팔이 과부하 · 재시도를 되풀이하지 않게.
	3. 팔 모터가 응답하지 않는다. state_timeout_s보다 짧으면 그 주기만 쉬고 명령 ·
	   램프를 이어 간다 (warnings). 그보다 길면 error.
	4. 드라이버의 팔 레지스터(lekiwi/motor/arm_registers)가 아직 없다 (idle) ·
	   보정 파일과 다르다 (error).
	5. arm_ready false · 목표 없음 · 목표 끊김 (idle, FollowerController).
	명령 tick은 tick으로도 한 번 더 자른다 (limit_ticks): 따라가는 중이면 지난 명령 tick ± follow_step_deg,
	아니면 마지막으로 잰 tick ± max_step_deg.
	rest (lekiwi/arm/rest)는 명령할 수 있을 때만 접은 자세 모드를 시작한다 (FollowerController).

	Attributes:
		controller: 램프 · 이동 제한 · 끊김 유지 (state · reason · diffs).
		warnings: 지난 step의 경고 (한국어). 명령을 막지 않는다.
	"""

	def __init__(
		self,
		converter: Any,
		controller: FollowerController,
		robot_id: str,
		state_timeout_s: float = 0.5,
		rest_pose_ticks: Sequence[int] = (),
	) -> None:
		"""설정을 받는다.

		Args:
			converter: ArmConverter (to_units · to_ticks · ranges ·
				homing_offsets).
			controller: 같은 converter의 limits · units_per_degree로 만든 것.
			robot_id: 보정 파일 이름 (상태 · 오류 글에 쓴다).
			state_timeout_s: 모터 상태가 끊기거나 팔 모터가 응답하지 않으면
				이만큼 [s] 뒤 오류.
			rest_pose_ticks: 접은 자세 [tick], 모터 ID 1~6 순서 (실물 tick, 보정으로
				LeRobot 단위로 바꾼다). 비우면 접은 자세가 없다 (rest를 거절한다).

		Raises:
			ValueError: state_timeout_s가 0 이하이거나, rest_pose_ticks가 목록이 아니거나
				정수 6개가 아니거나 0~4095 밖이다.
		"""
		if state_timeout_s <= 0.0:
			raise ValueError('state_timeout_s must be positive')
		if not isinstance(rest_pose_ticks, Sequence) or rest_pose_ticks and (
			len(rest_pose_ticks) != len(ARM_JOINTS)
			or any(
				isinstance(tick, bool) or not isinstance(tick, int) or not 0 <= tick <= MAX_TICK
				for tick in rest_pose_ticks)
		):
			raise ValueError(
				'rest_pose_ticks는 비우거나 모터 ID 1~6 순서의 정수 tick 6개(0~4095)여야 합니다: '
				f'{rest_pose_ticks!r}')
		self._rest_pose: Pose | None = converter.to_units(
			dict(zip(ARM_JOINTS, rest_pose_ticks))) if rest_pose_ticks else None
		self.controller = controller
		self.warnings: list[str] = []
		self._converter = converter
		self._robot_id = robot_id
		self._state_timeout_s = state_timeout_s
		self._max_step_ticks = round(controller.max_step_deg * MAX_TICK / 360.0)
		self._follow_step_ticks = round(controller.follow_step_deg * MAX_TICK / 360.0)
		self._sent_ticks: dict[str, int] = {}   # 마지막으로 보낸 tick
		self._calibration = {
			name: (converter.homing_offsets[name], *converter.ranges[name])
			for name in ARM_JOINTS}
		self._states_time: float | None = None
		self._are_arm_ids_right = False   # 마지막 모터 상태의 팔 ID가 lerobot LeKiwi와 같다
		self._ticks: dict[str, int] = {}     # 마지막으로 모두 응답한 팔 tick
		self._follower: Pose | None = None
		self._offline: list[str] = []
		self._offline_since = 0.0
		self._motor_error: str | None = None   # 걸어 둔 모터 이상
		self._current_motor_error: str | None = None   # 마지막 모터 상태의 이상
		self._is_hold_pending = False
		self._is_gripper_overloaded = False
		self._has_registers = False
		self._register_error: str | None = None
		self._register_warnings: list[str] = []
		self._is_arm_ready = False
		self._goal: Pose | None = None
		self._goal_time = 0.0

	def on_motor_states(self, now: float, motors: Sequence[Any]) -> Pose | None:
		"""lekiwi/motor/states (MotorState 목록).

		Returns:
			팔 모터가 모두 응답했으면 팔로워 자세 (lekiwi/arm/state), 아니면 None.
		"""
		arm = {motor.name: motor for motor in motors}
		self._states_time = now
		error = arm_motor_error(motors)
		# ID가 다르면 잡지 않는다: 이름으로 잰 tick을 다른 모터에 보내게 된다
		is_ids_right = all(motor.id == ARM_IDS[name] for name, motor in arm.items() if name in ARM_IDS)
		self._are_arm_ids_right = is_ids_right
		if error is not None and self._motor_error is None and self._is_arm_ready and is_ids_right:
			self._is_hold_pending = True
		self._current_motor_error = error
		# 팔이 켜져 있을 때만 걸어 둔다: 꺼져 있을 때 남은 비트가 다음 켜기를 막지 않게
		self._motor_error = error or (self._motor_error if self._is_arm_ready else None)
		gripper = arm.get(GRIPPER)
		self._is_gripper_overloaded = bool(
			gripper is not None and gripper.online and gripper.status & OVERLOAD_BIT)
		offline = [
			name for name in ARM_JOINTS
			if name not in arm or not arm[name].online]
		if offline and not self._offline:
			self._offline_since = now
		self._offline = offline
		if offline:
			self._follower = None
			return None
		self._ticks = {name: arm[name].position_ticks for name in ARM_JOINTS}
		self._follower = self._converter.to_units(self._ticks)
		return dict(self._follower)

	def on_goal(
		self,
		now: float,
		names: Sequence[str],
		positions: Sequence[float],
	) -> bool:
		"""리더 목표 (lekiwi/arm/goal). 팔 관절 6개가 모두 유한한 값이어야 받는다.

		Returns:
			받았으면 True.
		"""
		goal = dict(zip(names, positions))
		if not all(
			name in goal and math.isfinite(goal[name]) for name in ARM_JOINTS
		):
			return False
		self._goal = {name: float(goal[name]) for name in ARM_JOINTS}
		self._goal_time = now
		return True

	def on_arm_ready(self, is_ready: bool) -> None:
		"""lekiwi/motor/arm_ready: 드라이버가 팔 명령을 받는가."""
		self._is_arm_ready = is_ready
		# 드라이버는 바뀔 때만 보낸다: 받을 때마다 잰 자세에서 새로 출발한다
		# (false → true가 이어 와 false를 놓쳐도 램프 여부를 다시 본다).
		# 걸어 둔 이상은 풀되 마지막 모터 상태의 이상(ID 다름 포함)은 남긴다: 다음 상태까지 명령하지 않게
		self._motor_error = self._current_motor_error
		self._is_hold_pending = False
		self.controller.halt('팔 토크 상태가 바뀌었습니다', is_error=False)

	def on_registers(self, registers: Any) -> tuple[str | None, str | None]:
		"""lekiwi/motor/arm_registers (ArmRegisters): 보정 파일과 맞춰 본다.

		Returns:
			compare_registers의 (오류, 경고) (노드가 로그에 남긴다).
		"""
		self._has_registers = True
		self._register_error, warning = compare_registers(
			registers, self._calibration, self._robot_id)
		self._register_warnings = [] if warning is None else [warning]
		return self._register_error, warning

	@property
	def is_arm_confirmed(self) -> bool:
		"""마지막 모터 상태의 팔 ID가 lerobot LeKiwi(1~6)와 같고, 드라이버의 팔 레지스터를 받았고
		보정 파일과 맞다 (lerobot is_calibrated 기준).

		아니면 팔 LeRobot 단위 값이 다른 모터 · 다른 로봇의 보정으로 계산됐을 수 있다.
		"""
		return self._are_arm_ids_right and self._has_registers and self._register_error is None

	def rest(self, now: float) -> str | None:
		"""lekiwi/arm/rest: 잰 자세에서 접은 자세로 천천히 돌아가기 시작한다 (리더 목표는 무시).

		Returns:
			거절 이유 (한국어). 시작했으면 None.
		"""
		if not self._is_arm_ready:
			return '팔 토크가 꺼져 있습니다'
		reason, _ = self._block_reason(now)   # 상태 끊김 · 걸어 둔 모터 이상 · 레지스터 없음 · 보정 다름
		if reason is not None:
			return reason
		if self._rest_pose is None:
			return '접은 자세가 설정되지 않았습니다 (rest_pose_ticks)'
		if self._follower is None:
			return f'팔 모터 응답이 잠깐 끊겼습니다: {", ".join(self._offline)}'
		self.controller.start_rest(now, self._follower, self._rest_pose)
		return None

	def step(self, now: float) -> dict[str, int] | None:
		"""한 주기: 이번에 보낼 팔 tick (Goal_Position), 보내지 않으면 None."""
		controller = self.controller
		self.warnings = list(self._register_warnings)
		if self._is_gripper_overloaded:
			self.warnings.append('그리퍼 과부하: 모터가 스스로 힘을 줄입니다 (세게 쥐고 있습니다)')
		reason, is_error = self._block_reason(now)
		if reason is not None:
			controller.halt(reason, is_error)
			if self._is_hold_pending:   # 이상이 걸린 순간: 잰 자세로 한 번 잡는다
				self._is_hold_pending = False
				if not self._offline:   # 이번에 모두 잰 tick만 쓴다
					self._sent_ticks = dict(self._ticks)
					return dict(self._ticks)
			return None
		if self._offline:   # 잠깐 끊김: 이번 주기만 쉰다 (명령 · 램프는 그대로)
			self.warnings.append(
				f'팔 모터 응답이 잠깐 끊겨 이번 주기는 쉽니다: {", ".join(self._offline)}')
			return None
		command = controller.update(
			now, self._follower, self._goal, self._goal_time,
			self._is_arm_ready)
		if controller.warning is not None:
			self.warnings.append(controller.warning)
		if command is None:
			return None
		ticks = self._converter.to_ticks(command)
		if controller.is_step_from_command and self._sent_ticks:
			ticks = limit_ticks(ticks, self._sent_ticks, self._follow_step_ticks)
		else:
			ticks = limit_ticks(ticks, self._ticks, self._max_step_ticks)
		self._sent_ticks = ticks
		return dict(ticks)

	def status(self) -> dict[str, Any]:
		"""lekiwi/arm/status JSON 내용."""
		controller = self.controller
		return make_status(
			controller.state, controller.reason, self._robot_id,
			controller.diffs, controller.ramp_progress, self.warnings)

	def _block_reason(self, now: float) -> tuple[str | None, bool]:
		"""명령을 막는 (이유, 오류인가). 막지 않으면 (None, False)."""
		timeout = self._state_timeout_s
		if self._states_time is None or now - self._states_time > timeout:
			return (
				f'모터 상태가 {timeout:g}초 넘게 오지 않습니다 '
				'(lekiwi_motor_driver_node를 켜세요)', True)
		if (
			self._motor_error is not None and self._current_motor_error is None
			and now - self._goal_time > self.controller.goal_timeout_s
		):
			self._motor_error = None   # 이상이 풀렸고 목표가 끊겼다: 다음 목표는 잰 자세에서
		if self._motor_error is not None:
			return (
				f'{self._motor_error} (이상이 풀린 뒤 원격 조작을 다시 시작하면 이어 갑니다)',
				True)
		if self._offline and now - self._offline_since >= timeout:
			return f'팔 모터가 응답하지 않습니다: {", ".join(self._offline)}', True
		if not self._has_registers:
			return (
				'드라이버의 팔 모터 보정 값을 기다립니다 (lekiwi/motor/arm_registers)',
				False)
		if self._register_error is not None:
			return self._register_error, True
		return None, False
