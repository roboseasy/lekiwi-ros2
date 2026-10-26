"""ZMQ 조작 세션 상태기계 (ROS · zmq 없이, 시각을 인자로 받는다).

idle ─첫 유효 명령─▶ active (바퀴 토크 · cmd_vel 발행자) ─2 s 명령 없음─▶ idle (바퀴 토크 끔, 팔은 그대로)
  └─시작 거절─▶ blocked ◀─바깥에서 토크 꺼짐 · 비상 정지 · 토크 켜기 실패─ active (켠 바퀴 토크는 끈다)
blocked 는 명령을 버리고, 명령이 2 s 끊겨야 idle 로 돌아간다 (계속 보내는 클라이언트가 비상 정지를 저절로 풀지 못하게).
노드는 돌려받은 Effect 를 순서대로 실행한다.
"""

from dataclasses import dataclass
from typing import Any, Final

from lekiwi_zmq_host.protocol import Command

SESSION_IDLE: Final[str] = 'idle'
SESSION_ACTIVE: Final[str] = 'active'
SESSION_BLOCKED: Final[str] = 'blocked'

# [s] 명령이 이만큼 끊기면 cmd_vel 0 을 한 번 (베이스 컨트롤러는 0.3 s 에 이미 선다)
ZERO_TIMEOUT_S: Final[float] = 0.5
# [s] 명령이 이만큼 끊기면 세션 끝 (blocked 는 idle 로)
SESSION_TIMEOUT_S: Final[float] = 2.0

EXTERNAL_TORQUE_OFF: Final[str] = '바깥에서 토크가 꺼졌습니다 (비상 정지?)'
ESTOP_LATCHED: Final[str] = '비상 정지가 걸려 있습니다'

# Effect.kind
ENABLE_WHEELS: Final[str] = 'enable_wheels'        # lekiwi/motor/power true
ENABLE_ARM: Final[str] = 'enable_arm'              # lekiwi/motor/arm_torque true
DISABLE_WHEELS: Final[str] = 'disable_wheels'      # lekiwi/motor/power false
CREATE_CMD_VEL: Final[str] = 'create_cmd_vel'
DESTROY_CMD_VEL: Final[str] = 'destroy_cmd_vel'
PUBLISH_ZERO: Final[str] = 'publish_zero'          # cmd_vel 0
PUBLISH_ARM: Final[str] = 'publish_arm'            # value: {관절: lerobot 단위} → lekiwi/arm/goal
PUBLISH_BASE: Final[str] = 'publish_base'          # value: lerobot (x m/s, y m/s, theta deg/s) → cmd_vel
STATUS_CHANGED: Final[str] = 'status_changed'


@dataclass(frozen=True)
class Effect:
	"""노드가 할 일 하나."""

	kind: str
	value: Any = None


class Session:
	"""ZMQ 클라이언트 한 명의 조작 세션."""

	def __init__(self) -> None:
		self.state: str = SESSION_IDLE
		self.reason: str | None = None
		self._last_command_time: float | None = None
		self._is_zero_sent = False
		self._is_arm_enabled = False   # 이 세션에서 arm_torque true 를 요청했는가
		self._is_wheels_enabled = False   # 이 세션에서 power true 를 요청했고 아직 끄지 않았는가
		self._wheels_ready: bool | None = None
		self._arm_ready: bool | None = None
		# 이 세션에서 토크가 켜진 것을 보았는가 (그 뒤에 false 가 되면 바깥에서 끈 것)
		self._is_wheels_watched = False
		self._is_arm_watched = False

	def status(self) -> dict[str, Any]:
		return {'session': self.state, 'reason': self.reason}

	def _set(self, state: str, reason: str | None) -> list[Effect]:
		if (state, reason) == (self.state, self.reason):
			return []
		self.state, self.reason = state, reason
		return [Effect(STATUS_CHANGED)]

	def on_command(self, now: float, cmd: Command, blockers: list[str]) -> list[Effect]:
		"""명령 하나.

		Args:
			now: 지금 [s].
			cmd: 해석한 명령. base 가 None 이면 (명령 전체를 버림) 명령이 온 것으로 치지 않는다.
			blockers: 세션 시작을 막는 이유 (idle 에서만 본다).
		"""
		if cmd.base is None:
			if self.state == SESSION_BLOCKED:
				return []
			return self._set(self.state, cmd.problems[0] if cmd.problems else None)
		self._last_command_time = now
		if self.state == SESSION_BLOCKED:
			return []
		effects: list[Effect] = []
		if self.state == SESSION_IDLE:
			if blockers:
				return self._set(SESSION_BLOCKED, blockers[0])
			self._is_arm_enabled = False
			self._is_wheels_enabled = True
			self._is_wheels_watched = False
			self._is_arm_watched = False
			effects += [Effect(ENABLE_WHEELS), Effect(CREATE_CMD_VEL)]
		if cmd.arm is not None:
			if not self._is_arm_enabled:
				self._is_arm_enabled = True
				self._is_arm_watched = self._arm_ready is True
				effects.append(Effect(ENABLE_ARM))
			effects.append(Effect(PUBLISH_ARM, cmd.arm))
		effects.append(Effect(PUBLISH_BASE, cmd.base))
		self._is_zero_sent = False
		return effects + self._set(SESSION_ACTIVE, cmd.problems[0] if cmd.problems else None)

	def on_tick(self, now: float) -> list[Effect]:
		"""주기마다: 명령 끊김을 본다."""
		if self.state == SESSION_IDLE or self._last_command_time is None:
			return []
		silence = now - self._last_command_time
		if self.state == SESSION_BLOCKED:
			# 발행자는 막힐 때 지웠다: 남았을 때를 대비해 한 번 더 (지울 것이 없으면 노드가 무시)
			return [Effect(DESTROY_CMD_VEL), *self._set(SESSION_IDLE, None)] if silence >= SESSION_TIMEOUT_S else []
		if silence >= SESSION_TIMEOUT_S:
			self._is_wheels_enabled = False
			return [
				Effect(PUBLISH_ZERO), Effect(DESTROY_CMD_VEL), Effect(DISABLE_WHEELS),
				*self._set(SESSION_IDLE, None)]
		if silence >= ZERO_TIMEOUT_S and not self._is_zero_sent:
			self._is_zero_sent = True
			return [Effect(PUBLISH_ZERO)]
		return []

	def on_external(
		self, now: float, wheels_ready: bool | None, arm_ready: bool | None,
	) -> list[Effect]:
		"""lekiwi/motor/ready · arm_ready 를 받았을 때 (None = 이번에 안 바뀜).

		active 에서 켜진 것을 본 토크가 false 가 되면 blocked, 바로 cmd_vel 발행자를 지운다.
		바퀴는 세션 중에 받은 true 뒤부터 본다 (세션 시작 직후의 false 는 켜지는 중이다).
		팔은 arm_torque true 를 요청한 뒤부터: 이미 켜져 있었으면 그 값부터 (드라이버는 이미 켜진 팔에
		arm_ready 를 다시 내지 않는다), 아니면 받은 true 부터.
		"""
		del now
		if wheels_ready is not None:
			self._wheels_ready = wheels_ready
		if arm_ready is not None:
			self._arm_ready = arm_ready
		if self.state != SESSION_ACTIVE:
			return []
		# power true 는 늘 ready false (엔코더 기다림) → true 를 낸다: 세션 중에 받은 true 부터 본다
		if wheels_ready:
			self._is_wheels_watched = True
		if self._is_arm_enabled and self._arm_ready:
			self._is_arm_watched = True
		is_wheels_lost = self._is_wheels_watched and self._wheels_ready is False
		is_arm_lost = self._is_arm_watched and self._arm_ready is False
		if not (is_wheels_lost or is_arm_lost):
			return []
		return self._block(EXTERNAL_TORQUE_OFF)

	def on_estop(self, now: float, latched: bool) -> list[Effect]:
		"""lekiwi/safety/estop_latched 를 받았을 때: active 중 걸리면 blocked (idle 은 시작 거절이 맡는다).

		바퀴 ready 가 true 가 되기 전 (엔코더 기다림 · 실패) 에 걸려도 막는다.
		"""
		del now
		if not latched or self.state != SESSION_ACTIVE:
			return []
		return self._block(ESTOP_LATCHED)

	def on_failure(self, now: float, reason: str) -> list[Effect]:
		"""토크 켜기 서비스가 실패했을 때: active 면 blocked 로, 우리가 켠 바퀴 토크는 끈다."""
		del now
		if self.state != SESSION_ACTIVE:
			return []
		return self._block(reason)

	def _block(self, reason: str) -> list[Effect]:
		# 발행자를 남기면 게이트웨이가 바퀴 토크 켜기(= 비상 정지 풀기)를 거절한다
		effects = [Effect(PUBLISH_ZERO), Effect(DESTROY_CMD_VEL)]
		# 우리가 켠 바퀴 토크는 바로 끈다 (이미 꺼졌어도 power false 는 안전 · 멱등).
		# 2 s 뒤 idle 에서 끄면 그 사이 게이트웨이가 다시 켠 토크를 끌 수 있다.
		if self._is_wheels_enabled:
			self._is_wheels_enabled = False
			effects.append(Effect(DISABLE_WHEELS))
		return effects + self._set(SESSION_BLOCKED, reason)
