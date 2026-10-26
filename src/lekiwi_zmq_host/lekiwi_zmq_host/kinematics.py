"""lerobot 바퀴 명령 (x.vel · y.vel · theta.vel) → 베이스 컨트롤러 cmd_vel.

lerobot LeKiwi 는 몸체 속도를 자기 기구학(바퀴 반지름 0.05 · 몸체 반지름 0.125, 모터 7 왼 · 8 뒤 · 9 오른)으로
바퀴 속도 raw 로 바꿔 보냈다. 이 로봇은 cmd_vel 을 베이스 컨트롤러가 다른 기구학(omni_kinematics.cpp)으로
바퀴 속도로 바꾼다. 그래서 lerobot 식으로 바퀴 속도를 구한 뒤, 베이스 컨트롤러 기구학의 역으로
그 바퀴 속도를 내는 cmd_vel 을 구한다 — 같은 명령에서 바퀴가 옛 호스트와 같게 돈다.
lerobot 을 import 하지 않는다 (시험이 lerobot 원본과 대조한다).
"""

import math
from collections.abc import Mapping, Sequence
from typing import Final

# lerobot LeKiwi 바퀴 모터 ID: base_left_wheel, base_back_wheel, base_right_wheel
LEROBOT_WHEEL_IDS: Final[tuple[int, int, int]] = (7, 8, 9)
# STS3215 속도 raw 단위 (lerobot _degps_to_raw · _raw_to_degps)
STEPS_PER_DEG: Final[float] = 4096.0 / 360.0
SQRT3: Final[float] = math.sqrt(3.0)


def _degps_to_raw(degps: float) -> int:
	"""lerobot LeKiwi._degps_to_raw: 반올림 · 부호 있는 16비트로 자름."""
	speed = int(round(degps * STEPS_PER_DEG))
	return max(-0x8000, min(0x7FFF, speed))


def lerobot_body_to_wheel_raw(
	x_mps: float,
	y_mps: float,
	theta_degps: float,
	*,
	wheel_radius: float = 0.05,
	base_radius: float = 0.125,
	max_raw: int = 3000,
) -> dict[int, float]:
	"""lerobot LeKiwi._body_to_wheel_raw 와 같은 식 (max_raw 를 넘으면 세 바퀴를 같은 비율로 줄임까지).

	Args:
		x_mps: 앞 [m/s].
		y_mps: 왼쪽 [m/s].
		theta_degps: 반시계 [deg/s].
		wheel_radius: 바퀴 반지름 [m].
		base_radius: 몸체 중심 ↔ 바퀴 [m].
		max_raw: 바퀴 하나의 raw 상한.

	Returns:
		{모터 ID 7 · 8 · 9: raw 속도 (정수 값)}.
	"""
	theta_radps = theta_degps * (math.pi / 180.0)
	wheel_degps = []
	for mount_deg in (240.0, 0.0, 120.0):
		angle = math.radians(mount_deg - 90.0)
		linear = math.cos(angle) * x_mps + math.sin(angle) * y_mps + base_radius * theta_radps
		wheel_degps.append((linear / wheel_radius) * (180.0 / math.pi))
	peak_raw = max(abs(degps) * STEPS_PER_DEG for degps in wheel_degps)
	if peak_raw > max_raw:
		scale = max_raw / peak_raw
		wheel_degps = [degps * scale for degps in wheel_degps]
	return {
		motor_id: _degps_to_raw(degps)
		for motor_id, degps in zip(LEROBOT_WHEEL_IDS, wheel_degps)}


def wheel_raw_to_radps(raw: float) -> float:
	"""raw 속도 → 바퀴 각속도 [rad/s] (lerobot _raw_to_degps 를 rad/s 로)."""
	return math.radians(raw / STEPS_PER_DEG)


class BaseKinematics:
	"""베이스 컨트롤러 omni_kinematics.cpp 의 순 · 역기구학 (바퀴 상한 · 축소는 빼고).

	파라미터는 lekiwi_base_controller_node 와 같은 이름 · 뜻이다. wheel_ids 는 모델 바퀴 left · back · right
	순서의 모터 ID, wheel_directions 는 그 모터의 회전 방향 (모터 속도 = 모델 바퀴 속도 × 방향).

	Raises:
		ValueError: 값이 베이스 컨트롤러가 받지 않을 값이거나, 바퀴 모터가 lerobot 의 7 · 8 · 9 가 아님.
	"""

	def __init__(
		self,
		wheel_radius: float,
		base_radius: float,
		frame_x: float,
		frame_y: float,
		frame_yaw: float,
		wheel_ids: Sequence[int],
		wheel_directions: Sequence[float],
	) -> None:
		values = (wheel_radius, base_radius, frame_x, frame_y, frame_yaw, *wheel_directions)
		if not all(math.isfinite(value) for value in values):
			raise ValueError('베이스 기구학 값이 유한하지 않습니다')
		if wheel_radius <= 0.0 or base_radius <= 0.0:
			raise ValueError('wheel_radius · base_radius 는 0보다 커야 합니다')
		if sorted(wheel_ids) != list(LEROBOT_WHEEL_IDS):
			raise ValueError(f'wheel_ids 가 lerobot 바퀴 모터 7 · 8 · 9 가 아닙니다: {list(wheel_ids)}')
		if len(wheel_directions) != 3 or 0.0 in wheel_directions:
			raise ValueError(f'wheel_directions 는 0이 아닌 값 3개여야 합니다: {list(wheel_directions)}')
		self._wheel_radius = wheel_radius
		self._base_radius = base_radius
		self._frame_x = frame_x
		self._frame_y = frame_y
		self._cos = math.cos(frame_yaw)
		self._sin = math.sin(frame_yaw)
		self._wheel_ids = tuple(wheel_ids)
		self._wheel_directions = tuple(wheel_directions)

	def wheel_speeds(self, vx: float, vy: float, wz: float) -> dict[int, float]:
		"""cmd_vel (m/s, m/s, rad/s) → {모터 ID: 모터 속도 [rad/s]} (ToKinematicsFrame → ToRawWheelSpeeds)."""
		x = vx - wz * self._frame_y
		y = vy + wz * self._frame_x
		motor_x = self._cos * x + self._sin * y
		motor_lateral = -(-self._sin * x + self._cos * y)
		slots = [
			(-math.sin(angle) * motor_x + math.cos(angle) * motor_lateral + self._base_radius * wz)
			/ self._wheel_radius
			for angle in (2.0 * math.pi / 3.0, 0.0, 4.0 * math.pi / 3.0)]
		return {
			motor_id: speed * direction
			for motor_id, speed, direction in zip(self._wheel_ids, slots, self._wheel_directions)}

	def body_from_wheels(self, speeds: Mapping[int, float]) -> tuple[float, float, float]:
		"""{모터 ID: 모터 속도 [rad/s]} → cmd_vel (ToBodyTwist → FromKinematicsFrame, wheel_speeds 의 역)."""
		left, back, right = (
			speeds[motor_id] / direction * self._wheel_radius
			for motor_id, direction in zip(self._wheel_ids, self._wheel_directions))
		wz = (left + back + right) / (3.0 * self._base_radius)
		motor_x = (right - left) / SQRT3
		motor_y = -(back - self._base_radius * wz)
		return (
			self._cos * motor_x - self._sin * motor_y + wz * self._frame_y,
			self._sin * motor_x + self._cos * motor_y - wz * self._frame_x,
			wz)


def lerobot_action_to_cmd_vel(
	x: float, y: float, theta: float, base: BaseKinematics,
) -> tuple[float, float, float]:
	"""lerobot 바퀴 명령 (m/s, m/s, deg/s) → 같은 바퀴 속도를 내는 cmd_vel (m/s, m/s, rad/s)."""
	raw = lerobot_body_to_wheel_raw(x, y, theta)
	return base.body_from_wheels(
		{motor_id: wheel_raw_to_radps(value) for motor_id, value in raw.items()})
