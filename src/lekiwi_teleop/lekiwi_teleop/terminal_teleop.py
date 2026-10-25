"""터미널(Pi SSH 포함)에서 키를 누르고 있는 동안만 LeKiwi 베이스를 움직이는 조작 도구.

터미널은 키를 뗀 순간을 알려 주지 않는다. 키를 누르고 있으면 터미널이 같은 글자를 반복해서 보내므로,
그 반복 입력이 release_timeout 동안 끊기면 키를 뗀 것으로 보고 멈춘다.
release_timeout은 키보드 반복이 시작되기까지의 지연(보통 0.5 s)보다 길어야 한다.

방향 키 · 1/2/3 속도 · 가감속은 조작 창(lekiwi_guarded_base_teleop)과 같고, r/f로 속도를 한 칸씩 바꿀 수 있다.
시작할 때 lekiwi/motor/power true → lekiwi/motor/ready true를 확인하고,
끝날 때(Esc, Ctrl+C, 시간 초과, SSH 끊김, 오류) zero cmd_vel과 lekiwi/motor/power false를 보낸다.
"""

import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from typing import Callable, Sequence

import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions

from lekiwi_teleop.guarded_base_teleop import (
	GuardedTeleopConfig,
	HeldKeySpeedProfile,
	_handle_cleanup_failure,
)
from lekiwi_teleop.interface_names import (
	BASE_CONTROLLER_NODE,
	MOTOR_READY_TOPIC,
	qualified_node_name,
)
from lekiwi_teleop.motor_session import Motion, RosMotorEndpoint, cleanup_session

DEFAULT_RELEASE_TIMEOUT = 0.6     # [s] 반복 입력이 이만큼 끊기면 뗀 것으로 본다
MIN_RELEASE_TIMEOUT = 0.2
MAX_RELEASE_TIMEOUT = 1.5
PUBLISH_PERIOD = 0.05             # [s] cmd_vel 발행 주기 (조작 창과 같다)

KEY_EXIT = "exit"                 # Esc
KEY_IGNORED = "ignored"           # 방향키(화살표) 같은 이스케이프 시퀀스
KEY_CTRL_C = "\x03"

SPEED_CHANGE_INTERVAL = 0.2      # [s] r/f를 누르고 있으면 이 간격으로 한 칸씩 바뀐다

# 이 조작 도구들(터미널 · 키보드 창)의 자체 상한. 베이스 컨트롤러 상한과 둘 중 작은 값을 쓴다.
# 2026-10-22 베이스 상한을 텔레오퍼레이션 화면용으로 0.23 m/s · 1.53 rad/s로 올렸지만, 이 도구들은
# 예전 베이스 상한(0.11 · 0.1 · 0.11 m/s 중 최소 0.1, 0.75 rad/s)에서 쓰던 최고 단계(0.10 m/s · 0.58 rad/s)를
# 그대로 쓴다. 더 빠르게 하려면 --max-linear · --max-angular로 올린다.
DEFAULT_MAX_LINEAR = 0.10         # [m/s]
DEFAULT_MAX_ANGULAR = 0.75        # [rad/s]

HELP_TEXT = (
	"LeKiwi 터미널 조작 · 키를 누르고 있는 동안만 이동\r\n"
	"  w/s 앞 · 뒤   a/d 왼쪽 · 오른쪽   q/e 좌회전 · 우회전\r\n"
	"  r/f 속도 한 칸 올림 · 내림   1/2/3 속도 0.04 · 0.05 · 0.06 m/s\r\n"
	"  Space 정지   Esc 또는 Ctrl+C 종료\r\n"
)


class TerminalSpeedProfile(HeldKeySpeedProfile):
	"""조작 창의 1/2/3 단계에 r/f로 한 칸씩 올리고 내리는 속도 단계를 더한다.

	0.01 ~ 0.20 m/s를 0.01 간격 20단계로 나누고, 회전 속도는 조작 창 단계와 같은 비율(약 5.8 rad/m)로 따라간다.
	베이스 컨트롤러 상한을 넘는 단계는 쓰지 않는다 (max_linear, max_angular로 준다).
	"""

	LEVELS = (   # (선속도 m/s, 회전 rad/s)
		(0.01, 0.06), (0.02, 0.12), (0.03, 0.17), (0.04, 0.23), (0.05, 0.29),
		(0.06, 0.35), (0.07, 0.41), (0.08, 0.46), (0.09, 0.52), (0.10, 0.58),
		(0.11, 0.64), (0.12, 0.70), (0.13, 0.75), (0.14, 0.81), (0.15, 0.87),
		(0.16, 0.93), (0.17, 0.99), (0.18, 1.04), (0.19, 1.10), (0.20, 1.16),
	)
	PRESET_LEVELS = {"1": 3, "2": 4, "3": 5}   # 조작 창 1/2/3단과 같은 속도 (0.04 / 0.05 / 0.06 m/s)

	def __init__(self, max_linear: float = math.inf, max_angular: float = math.inf):
		super().__init__()
		# 상한 안에 드는 가장 빠른 단계. 1/2/3단(0.06 m/s)보다 낮은 상한은 받지 않는다.
		usable = [
			index for index, (linear_speed, angular_speed) in enumerate(self.LEVELS)
			if linear_speed <= max_linear + 1e-9 and angular_speed <= max_angular + 1e-9
		]
		if not usable or usable[-1] < max(self.PRESET_LEVELS.values()):
			raise ValueError(
				f"base speed limits ({max_linear:g} m/s, {max_angular:g} rad/s) are below teleop stage 3")
		self.max_level = usable[-1]
		self.level = self.PRESET_LEVELS["1"]

	def press(self, key: str) -> bool:
		if key in self.PRESET_LEVELS:
			self.level = self.PRESET_LEVELS[key]
			return super().press(key)
		if key == "r":
			self.level = min(self.max_level, self.level + 1)
			return True
		if key == "f":
			self.level = max(0, self.level - 1)
			return True
		return super().press(key)

	def current_speeds(self) -> tuple[float, float]:
		return self.LEVELS[self.level]


class TerminalHoldControl:
	"""터미널 키 입력을 '누르고 있는 방향 키'로 바꾼다.

	방향 키가 들어오면 누른 것으로, 그 키의 반복 입력이 release_timeout 동안 끊기면 뗀 것으로 본다.
	Space를 누를 때 누르고 있던 키는 손을 뗄 때까지(반복이 끊길 때까지) 무시해 다시 움직이지 않게 한다.
	"""

	def __init__(
		self, release_timeout: float, clock: Callable[[], float] = time.monotonic,
		max_linear: float = math.inf, max_angular: float = math.inf,
	):
		self.profile = TerminalSpeedProfile(max_linear, max_angular)
		self.release_timeout = release_timeout
		self.clock = clock
		self.last_seen: dict[str, float] = {}   # 누르고 있는 방향 키 → 마지막 입력 시각
		self.blocked: dict[str, float] = {}     # Space 뒤 무시하는 키 → 마지막 입력 시각
		self.last_speed_change: float | None = None

	def handle_key(self, key: str) -> bool:
		"""키 하나를 처리한다. 바로 zero cmd_vel을 보내야 하면 True."""
		now = self.clock()
		key = key.lower()
		if key == " ":
			for held in self.profile.held_keys:
				self.blocked[held] = self.last_seen.get(held, now)
			self.last_seen.clear()
			self.profile.press("space")
			return True
		if key in ("1", "2", "3"):
			self.profile.press(key)
			return False
		if key in ("r", "f"):
			# 누르고 있으면 반복 입력이 1초에 수십 번 오므로 SPEED_CHANGE_INTERVAL마다 한 칸만 바꾼다
			if self.last_speed_change is None or now - self.last_speed_change >= SPEED_CHANGE_INTERVAL:
				self.profile.press(key)
				self.last_speed_change = now
			return False
		if key not in HeldKeySpeedProfile.DIRECTIONS:
			return False
		if key in self.blocked:
			self.blocked[key] = now
			return False
		self.last_seen[key] = now
		self.profile.press(key)
		return False

	def expire(self) -> bool:
		"""반복 입력이 끊긴 키를 뗀다. 움직이던 키를 떼서 바로 zero를 보내야 하면 True."""
		now = self.clock()
		has_stopped = False
		for key, seen in list(self.last_seen.items()):
			if now - seen > self.release_timeout:
				del self.last_seen[key]
				was_active = self.profile.active_key == key
				self.profile.release(key)
				has_stopped = has_stopped or (was_active and self.profile.active_key is None)
		for key, seen in list(self.blocked.items()):
			if now - seen > self.release_timeout:
				del self.blocked[key]
		return has_stopped

	def status(self) -> str:
		linear_speed, angular_speed = self.profile.current_speeds()
		top_speed = self.profile.LEVELS[self.profile.max_level][0]
		return (
			f"속도 {linear_speed:.2f} m/s (최고 {top_speed:.2f}) · 회전 {angular_speed:.2f} rad/s · "
			f"{self.profile.active_key or '정지'}"
		)


class TerminalKeyInput:
	"""raw 모드 터미널에서 키 하나씩 읽는다. Esc와 방향키 시퀀스(ESC [ A 등)를 구분한다."""

	ESCAPE_SEQUENCE_WAIT = 0.02   # [s] Esc 뒤에 바로 이어지는 바이트가 있으면 시퀀스로 본다

	def __init__(self, stream=None):
		self.stream = stream if stream is not None else sys.stdin
		self.fd = self.stream.fileno()
		self.settings = None

	def __enter__(self):
		self.settings = termios.tcgetattr(self.fd)
		tty.setraw(self.fd)
		return self

	def __exit__(self, exc_type, exc_value, traceback):
		if self.settings is not None:
			termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

	def _ready(self, timeout_sec: float) -> bool:
		readable, _, _ = select.select([self.fd], [], [], timeout_sec)
		return bool(readable)

	def read_key(self, timeout_sec: float) -> str | None:
		if not self._ready(timeout_sec):
			return None
		data = os.read(self.fd, 1)
		if not data:
			raise EOFError("terminal input closed")
		key = data.decode(errors="ignore")
		if key != "\x1b":
			return key
		if not self._ready(self.ESCAPE_SEQUENCE_WAIT):
			return KEY_EXIT
		os.read(self.fd, 8)   # 방향키 등 시퀀스의 나머지는 버린다
		return KEY_IGNORED


def run_terminal_session(
	endpoint,
	read_key: Callable[[float], str | None],
	*,
	config: GuardedTeleopConfig = GuardedTeleopConfig(),
	release_timeout: float = DEFAULT_RELEASE_TIMEOUT,
	speed_limits: tuple[float, float] = (math.inf, math.inf),
	clock: Callable[[], float] = time.monotonic,
	status: Callable[[str], None] = lambda message: None,
	report_cleanup_error: Callable[[str], None] = lambda message: print(message, file=sys.stderr),
) -> str:
	"""토크를 켜고 키 입력으로 cmd_vel을 보낸다. 어떻게 끝나든 zero와 토크 끄기를 시도한다.

	speed_limits: 베이스 컨트롤러의 (선속도 상한 m/s, 회전 상한 rad/s). 이를 넘는 속도 단계는 쓰지 않는다.
	"""
	deadline = clock() + config.max_session_duration
	control = TerminalHoldControl(release_timeout, clock, *speed_limits)
	primary_error: BaseException | None = None
	try:
		for _ in range(config.zero_cycles):
			endpoint.publish_motion(Motion())
		remaining = deadline - clock()
		if remaining <= 0.0:
			raise RuntimeError("maximum session duration expired during preparation")
		status("모터 켜는 중")
		endpoint.set_motor_power(True, timeout_sec=remaining)
		endpoint.begin_ready_observation()
		remaining = deadline - clock()
		if remaining <= 0.0:
			raise RuntimeError("maximum session duration expired during preparation")
		ready_timeout = min(config.ready_timeout, remaining)
		if not endpoint.wait_for_motor_ready(ready_timeout):
			raise RuntimeError(
				f"{MOTOR_READY_TOPIC} did not produce a true sample within {ready_timeout:g}s"
			)

		last_status = None
		while True:
			remaining = deadline - clock()
			if remaining <= 0.0:
				return "timeout"
			key = read_key(min(PUBLISH_PERIOD, remaining))
			if key == KEY_CTRL_C:
				raise KeyboardInterrupt
			if key == KEY_EXIT:
				return "exit"
			if key is not None and key != KEY_IGNORED and control.handle_key(key):
				endpoint.publish_motion(Motion())
			if control.expire():
				endpoint.publish_motion(Motion())
			endpoint.spin_once()
			if endpoint.motor_ready is False:
				raise RuntimeError(f"{MOTOR_READY_TOPIC} became false during teleop")
			if deadline - clock() <= 0.0:
				return "timeout"
			endpoint.publish_motion(control.profile.next_motion())
			current_status = control.status()
			if current_status != last_status:
				status(current_status)
				last_status = current_status
	except BaseException as error:
		primary_error = error
		raise
	finally:
		cleanup_error = cleanup_session(endpoint, config.zero_cycles)
		if cleanup_error is not None:
			_handle_cleanup_failure(cleanup_error, primary_error, report_cleanup_error)


def positive_finite_float(value: str) -> float:
	parsed = float(value)
	if not math.isfinite(parsed) or parsed <= 0.0:
		raise argparse.ArgumentTypeError(f"finite value greater than zero required: {value}")
	return parsed


def release_timeout_value(value: str) -> float:
	parsed = positive_finite_float(value)
	if not MIN_RELEASE_TIMEOUT <= parsed <= MAX_RELEASE_TIMEOUT:
		raise argparse.ArgumentTypeError(
			f"release timeout must be in [{MIN_RELEASE_TIMEOUT}, {MAX_RELEASE_TIMEOUT}] s: {value}"
		)
	return parsed


def add_speed_limit_arguments(parser: argparse.ArgumentParser) -> None:
	parser.add_argument(
		"--max-linear",
		type=positive_finite_float,
		default=DEFAULT_MAX_LINEAR,
		help=f"Linear speed cap in m/s, also capped by the base controller (default: {DEFAULT_MAX_LINEAR}).",
	)
	parser.add_argument(
		"--max-angular",
		type=positive_finite_float,
		default=DEFAULT_MAX_ANGULAR,
		help=f"Yaw rate cap in rad/s, also capped by the base controller (default: {DEFAULT_MAX_ANGULAR}).",
	)


def cap_speed_limits(base_limits: tuple[float, float], args: argparse.Namespace) -> tuple[float, float]:
	"""베이스 컨트롤러 상한과 이 도구의 상한(--max-linear, --max-angular) 중 작은 값."""
	return min(base_limits[0], args.max_linear), min(base_limits[1], args.max_angular)


def build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(
		description="Hold-to-move terminal control for LeKiwi (works over SSH)."
	)
	parser.add_argument(
		"--yes-i-confirm-motion",
		action="store_true",
		required=True,
		help="Acknowledge that fresh on-site motion authorization was obtained.",
	)
	parser.add_argument(
		"--max-session-duration",
		type=positive_finite_float,
		default=3600.0,
		help="Maximum total session duration in seconds (default: 3600).",
	)
	parser.add_argument(
		"--release-timeout",
		type=release_timeout_value,
		default=DEFAULT_RELEASE_TIMEOUT,
		help=(
			"Seconds without key repeat before a key counts as released "
			f"(default: {DEFAULT_RELEASE_TIMEOUT}, must exceed the keyboard repeat delay)."
		),
	)
	add_speed_limit_arguments(parser)
	return parser


def read_base_speed_limits(node, timeout_sec: float = 3.0) -> tuple[float, float]:
	"""베이스 컨트롤러의 속도 상한을 읽는다: (선속도 m/s, 회전 rad/s). 읽지 못하면 RuntimeError."""
	base_node = qualified_node_name(node.get_namespace(), BASE_CONTROLLER_NODE)
	client = AsyncParameterClient(node, base_node)
	if not client.wait_for_services(timeout_sec=timeout_sec):
		raise RuntimeError(f"{base_node} parameter services are unavailable; is the base controller running?")
	names = ["max_linear_x", "max_linear_y", "max_linear_speed", "max_angular_z"]
	future = client.get_parameters(names)
	rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
	if not future.done() or future.result() is None:
		raise RuntimeError(f"could not read {base_node} speed limits")
	values = dict(zip(names, map(parameter_value_to_python, future.result().values)))
	if any(not isinstance(value, (int, float)) for value in values.values()):
		raise RuntimeError(f"{base_node} speed limits are missing: {values}")
	max_linear = min(values["max_linear_x"], values["max_linear_y"], values["max_linear_speed"])
	return float(max_linear), float(values["max_angular_z"])


def write_status(message: str) -> None:
	"""raw 모드라 줄바꿈 없이 같은 줄을 덮어쓴다. 터미널이 닫혀도 예외를 내지 않는다."""
	try:
		sys.stdout.write(f"\r\x1b[K{message}")
		sys.stdout.flush()
	except OSError:
		pass


def raise_keyboard_interrupt(_signum, _frame):
	raise KeyboardInterrupt


def main(argv: Sequence[str] | None = None) -> int:
	args = build_parser().parse_args(argv)
	if not args.yes_i_confirm_motion:
		raise ValueError("--yes-i-confirm-motion is required")
	if not sys.stdin.isatty():
		print("terminal teleop needs an interactive terminal (stdin is not a TTY)", file=sys.stderr)
		return 2
	config = GuardedTeleopConfig(max_session_duration=args.max_session_duration)

	# SSH가 끊기거나(SIGHUP) 종료 신호(SIGTERM)를 받아도 정지 · 토크 끄기를 거친다.
	signal.signal(signal.SIGHUP, raise_keyboard_interrupt)
	signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
	rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
	endpoint = None
	session_started = False
	primary_error: BaseException | None = None
	try:
		endpoint = RosMotorEndpoint(node_name="lekiwi_terminal_teleop")
		endpoint.initialize()
		speed_limits = cap_speed_limits(read_base_speed_limits(endpoint.node), args)
		endpoint.node.get_logger().info(
			f"speed limits: {speed_limits[0]:g} m/s, {speed_limits[1]:g} rad/s "
			"(base controller and --max-linear/--max-angular; faster teleop steps are not used)"
		)
		endpoint.node.get_logger().warning(
			f"Hold-to-move terminal teleop for at most {config.max_session_duration:g}s "
			f"(release after {args.release_timeout:g}s without key repeat): "
			"w/s x, a/d y, q/e yaw, r/f speed step, 1/2/3 speed preset, Space stop, Esc exit"
		)
		print(HELP_TEXT.replace("\r\n", "\n"), end="", flush=True)
		with TerminalKeyInput() as terminal:
			session_started = True
			result = run_terminal_session(
				endpoint, terminal.read_key, config=config, release_timeout=args.release_timeout,
				speed_limits=speed_limits, status=write_status,
				report_cleanup_error=endpoint.report_cleanup_error,
			)
		print()
		endpoint.node.get_logger().info(f"terminal teleop ended ({result}); wheel torque disabled")
		return 0
	except KeyboardInterrupt as error:
		primary_error = error
		print()
		return 130
	except Exception as error:
		primary_error = error
		print(f"\nterminal teleop failed: {error}", file=sys.stderr)
		return 1
	finally:
		try:
			if endpoint is not None:
				if not session_started:
					cleanup_error = cleanup_session(endpoint, config.zero_cycles)
					if cleanup_error is not None:
						_handle_cleanup_failure(
							cleanup_error, primary_error, endpoint.report_cleanup_error)
				endpoint.destroy()
		finally:
			rclpy.try_shutdown()


if __name__ == "__main__":
	raise SystemExit(main())
