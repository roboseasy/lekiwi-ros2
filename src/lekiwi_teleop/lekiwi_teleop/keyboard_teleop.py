"""키보드 동시 입력으로 LeKiwi 베이스를 움직이는 조작 창 (w+d 대각선, w+q 앞으로 가며 회전).

창이 키를 누른 순간과 뗀 순간을 따로 받아, 누르고 있는 방향 키를 모두 합쳐 cmd_vel로 보낸다.
Pi에서 실행하고 `ssh -X`로 접속하면 창이 노트북 화면에 뜬다 (노트북에 ROS가 없어도 된다).

조작 창(lekiwi_guarded_base_teleop)의 안전 처리를 그대로 쓴다:
시작할 때 lekiwi/motor/power true → lekiwi/motor/ready true 확인, 창 밖을 누르면 정지,
ready가 false가 되면 종료, 끝날 때(Esc · 창 닫기 · 시간 초과 · 오류) zero cmd_vel과 lekiwi/motor/power false.
"""

import math
import sys
import time
from typing import Callable, Sequence

import rclpy
from rclpy.signals import SignalHandlerOptions

from lekiwi_teleop.guarded_base_teleop import (
	GuardedTeleopConfig,
	HoldTeleopWindow,
	_handle_cleanup_failure,
	build_parser,
	config_from_args,
	run_hold_gui_session,
)
from lekiwi_teleop.motor_session import Motion, RosMotorEndpoint, cleanup_session
from lekiwi_teleop.terminal_teleop import (
	TerminalSpeedProfile,
	add_speed_limit_arguments,
	cap_speed_limits,
	read_base_speed_limits,
)


def _approach(output: float, target: float, step: float) -> float:
	if target > output:
		return min(target, output + step)
	if target < output:
		return max(target, output - step)
	return target


class CombinedKeySpeedProfile:
	"""누르고 있는 방향 키를 모두 합친다.

	w/s(앞뒤)와 a/d(좌우)를 더한 방향으로 그 단계의 선속도만큼 간다 (대각선도 같은 속도).
	q/e(회전)는 이동과 함께 쓸 수 있다. 반대 키를 함께 누르면 그 축은 0이다.
	방향 키를 모두 떼면 바로 0이다. 그 밖에는 50 ms마다 LINEAR_STEP · ANGULAR_STEP씩 목표에 다가간다.
	"""

	DIRECTIONS = {   # (x, y, yaw) 부호
		"w": (1.0, 0.0, 0.0),
		"s": (-1.0, 0.0, 0.0),
		"a": (0.0, 1.0, 0.0),
		"d": (0.0, -1.0, 0.0),
		"q": (0.0, 0.0, 1.0),
		"e": (0.0, 0.0, -1.0),
	}
	LEVELS = TerminalSpeedProfile.LEVELS        # 0.01 ~ 0.20 m/s, 20단계
	MIN_TOP_LEVEL = 4                            # 0.05 m/s. 베이스 상한이 이보다 낮으면 시작하지 않는다
	START_LEVEL = 9                              # 0.10 m/s
	LINEAR_STEP = 0.02                           # [m/s] 50 ms마다 → 0.20 m/s까지 0.5 s
	ANGULAR_STEP = 0.12                          # [rad/s] 50 ms마다
	SPEED_CHANGE_INTERVAL = 0.2                  # [s] r/f를 누르고 있으면 이 간격으로 한 칸

	def __init__(
		self, max_linear: float = math.inf, max_angular: float = math.inf,
		clock: Callable[[], float] = time.monotonic,
	):
		usable = [
			index for index, (linear_speed, angular_speed) in enumerate(self.LEVELS)
			if linear_speed <= max_linear + 1e-9 and angular_speed <= max_angular + 1e-9
		]
		if not usable or usable[-1] < self.MIN_TOP_LEVEL:
			raise ValueError(
				f"base speed limits ({max_linear:g} m/s, {max_angular:g} rad/s) are too low for keyboard teleop")
		self.max_level = usable[-1]
		self.level = min(self.START_LEVEL, self.max_level)
		self.clock = clock
		self.held_keys: list[str] = []
		self.control = Motion()
		self.last_speed_change: float | None = None

	@property
	def active_key(self) -> str | None:
		return "+".join(self.held_keys) if self.held_keys else None

	def current_speeds(self) -> tuple[float, float]:
		return self.LEVELS[self.level]

	def press(self, key: str) -> bool:
		if key in ("r", "f"):
			now = self.clock()
			if self.last_speed_change is None or now - self.last_speed_change >= self.SPEED_CHANGE_INTERVAL:
				step = 1 if key == "r" else -1
				self.level = min(self.max_level, max(0, self.level + step))
				self.last_speed_change = now
			return True
		if key == "space":
			self.stop()
			return True
		if key not in self.DIRECTIONS or key in self.held_keys:
			return False
		self.held_keys.append(key)
		return True

	def release(self, key: str) -> bool:
		"""키를 뗀다. 모든 방향 키를 떼서 바로 zero를 보내야 하면 True."""
		if key not in self.held_keys:
			return False
		self.held_keys.remove(key)
		if not self.held_keys:
			self.control = Motion()
			return True
		return False

	def stop(self) -> None:
		self.held_keys.clear()
		self.control = Motion()

	def target(self) -> Motion:
		x = sum(self.DIRECTIONS[key][0] for key in self.held_keys)
		y = sum(self.DIRECTIONS[key][1] for key in self.held_keys)
		yaw = sum(self.DIRECTIONS[key][2] for key in self.held_keys)
		linear_speed, angular_speed = self.current_speeds()
		norm = math.hypot(x, y)
		return Motion(
			linear_x=x / norm * linear_speed if norm > 0.0 else 0.0,
			linear_y=y / norm * linear_speed if norm > 0.0 else 0.0,
			angular_z=math.copysign(angular_speed, yaw) if yaw != 0.0 else 0.0,
		)

	def next_motion(self) -> Motion:
		if not self.held_keys:
			self.control = Motion()
			return self.control
		target = self.target()
		self.control = Motion(
			linear_x=_approach(self.control.linear_x, target.linear_x, self.LINEAR_STEP),
			linear_y=_approach(self.control.linear_y, target.linear_y, self.LINEAR_STEP),
			angular_z=_approach(self.control.angular_z, target.angular_z, self.ANGULAR_STEP),
		)
		return self.control

	def status_text(self) -> str:
		linear_speed, angular_speed = self.current_speeds()
		top_speed = self.LEVELS[self.max_level][0]
		return (
			f"속도 {linear_speed:.2f} m/s (최고 {top_speed:.2f}) · 회전 {angular_speed:.2f} rad/s · "
			f"{self.active_key or '정지'}"
		)


class KeyboardTeleopWindow(HoldTeleopWindow):
	"""조작 창의 키 처리(뗌 확인 · 창 밖 클릭 정지 · Space 차단)에 동시 입력 속도 계산을 붙인다."""

	# 원격(ssh -X)에서는 키 반복의 뗌 · 눌림 쌍이 조금 벌어져 올 수 있어 조작 창(35 ms)보다 길게 둔다
	RELEASE_DEBOUNCE_MS = 60

	def __init__(self, root, endpoint, deadline, clock, status, profile):
		super().__init__(root, endpoint, deadline, clock, status)
		self.profile = profile

	def update_status(self):
		self.status(self.profile.status_text())

	def tick(self):
		super().tick()
		if not self.closed:
			self.update_status()


def main(argv: Sequence[str] | None = None) -> int:
	parser = build_parser()
	parser.description = "Hold-to-move keyboard window for LeKiwi with simultaneous keys (w+d diagonal)."
	add_speed_limit_arguments(parser)
	args = parser.parse_args(argv)
	config = config_from_args(args)
	endpoint = None
	root = None
	session_started = False
	primary_error: BaseException | None = None
	rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
	try:
		import tkinter as tk

		root = tk.Tk()
		root.title("LeKiwi 키보드 조작 · 동시 입력")
		root.geometry("520x220")
		root.resizable(False, False)
		tk.Label(root, text="W/S 앞뒤 · A/D 좌우 · Q/E 회전 (함께 누르면 합쳐서)", font=("Sans", 14)).pack(pady=(20, 6))
		tk.Label(
			root, text="R/F 속도 한 칸 올림 · 내림 · Space 정지 · Esc 종료",
			font=("Sans", 10)).pack(pady=4)
		tk.Label(root, text="키를 놓거나 창 밖을 누르면 정지", font=("Sans", 10)).pack(pady=2)
		status_text = tk.StringVar(value="모터 준비 중")
		tk.Label(root, textvariable=status_text, font=("Sans", 13)).pack(pady=12)
		root.update()
		endpoint = RosMotorEndpoint(node_name="lekiwi_keyboard_teleop")
		endpoint.initialize()
		max_linear, max_angular = cap_speed_limits(read_base_speed_limits(endpoint.node), args)
		profile = CombinedKeySpeedProfile(max_linear, max_angular)
		endpoint.node.get_logger().warning(
			f"Keyboard teleop for at most {config.max_session_duration:g}s, speed limits "
			f"{max_linear:g} m/s {max_angular:g} rad/s: w/s x, a/d y, q/e yaw (combinable), "
			"r/f speed step, Space stop, Esc exit"
		)
		session_started = True
		run_hold_gui_session(
			endpoint, root, config=config, status=status_text.set,
			window_factory=lambda root_, endpoint_, deadline, clock, status: KeyboardTeleopWindow(
				root_, endpoint_, deadline, clock, status, profile),
		)
		endpoint.node.get_logger().info("keyboard teleop ended; wheel torque disabled")
		return 0
	except KeyboardInterrupt as error:
		primary_error = error
		return 130
	except Exception as error:
		primary_error = error
		print(f"keyboard teleop failed: {error}", file=sys.stderr)
		return 1
	finally:
		try:
			if endpoint is not None:
				if not session_started:
					cleanup_error = cleanup_session(endpoint, GuardedTeleopConfig().zero_cycles)
					if cleanup_error is not None:
						_handle_cleanup_failure(cleanup_error, primary_error, endpoint.report_cleanup_error)
				endpoint.destroy()
			if root is not None:
				root.destroy()
		finally:
			rclpy.try_shutdown()


if __name__ == "__main__":
	raise SystemExit(main())
