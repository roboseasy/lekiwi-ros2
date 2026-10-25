"""터미널 teleop 시험: 반복 입력으로 '누르고 있음'을 판단하는 규칙, 세션 순서, 키 읽기. ROS · 하드웨어 없음."""

import contextlib
import io
import os
import unittest

from lekiwi_teleop.guarded_base_teleop import GuardedTeleopConfig
from lekiwi_teleop.motor_session import Motion
from lekiwi_teleop.terminal_teleop import (
	KEY_CTRL_C,
	KEY_EXIT,
	KEY_IGNORED,
	TerminalHoldControl,
	TerminalKeyInput,
	build_parser,
	cap_speed_limits,
	run_terminal_session,
)


class FakeClock:
	def __init__(self):
		self.now = 100.0

	def __call__(self):
		return self.now


class FakeEndpoint:
	def __init__(self, *, ready=True):
		self.ready = ready
		self.motor_ready = None
		self.events = []
		self.ready_after_publishes = None   # 이만큼 발행한 뒤 ready를 false로 바꾼다

	def publish_motion(self, motion):
		self.events.append(("publish", motion))
		publishes = sum(1 for event in self.events if event[0] == "publish")
		if self.ready_after_publishes is not None and publishes >= self.ready_after_publishes:
			self.motor_ready = False

	def set_motor_power(self, enabled, timeout_sec=None):
		self.events.append(("power", enabled))

	def begin_ready_observation(self):
		self.events.append(("begin_ready",))

	def wait_for_motor_ready(self, timeout_sec):
		self.events.append(("ready", timeout_sec))
		if self.ready:
			self.motor_ready = True
		return self.ready

	def spin_once(self):
		pass

	def publishes(self):
		return [event[1] for event in self.events if event[0] == "publish"]


class KeyScript:
	"""read_key 대신 쓴다. (경과 시간, 키) 목록을 차례로 돌려주고 가짜 시계를 움직인다."""

	def __init__(self, clock, steps):
		self.clock = clock
		self.steps = list(steps)

	def __call__(self, timeout_sec):
		if not self.steps:
			return KEY_EXIT
		elapsed, key = self.steps.pop(0)
		self.clock.now += elapsed
		return key


def held(key, duration, period=0.05):
	"""키를 duration 동안 누르고 있을 때 터미널이 보내는 반복 입력 (반복 시작 지연 0.5 s 포함)."""
	steps = [(period, key)]
	elapsed = 0.5
	steps.append((0.5, key))
	while elapsed + period <= duration:
		steps.append((period, key))
		elapsed += period
	return steps


def idle(duration, period=0.05):
	return [(period, None)] * int(round(duration / period))


class TestParser(unittest.TestCase):
	def test_requires_motion_confirmation(self):
		with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
			build_parser().parse_args([])

	def test_release_timeout_bounds(self):
		self.assertEqual(
			build_parser().parse_args(["--yes-i-confirm-motion"]).release_timeout, 0.6)
		for bad in ("0.1", "2.0", "nan", "-1"):
			with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
				build_parser().parse_args(["--yes-i-confirm-motion", "--release-timeout", bad])

	def test_speed_caps_keep_the_old_top_speed_by_default(self):
		# 2026-10-22 베이스 상한은 0.23 m/s · 1.53 rad/s지만, 이 도구는 기본으로 예전 최고 단계(0.10 m/s)를 쓴다
		args = build_parser().parse_args(["--yes-i-confirm-motion"])
		self.assertEqual(cap_speed_limits((0.23, 1.53), args), (0.10, 0.75))
		control = TerminalHoldControl(0.6, FakeClock(), *cap_speed_limits((0.23, 1.53), args))
		self.assertIn("(최고 0.10)", control.status())
		# 직접 올리면 베이스 상한 안에서 더 빠른 단계를 쓴다
		raised = build_parser().parse_args(
			["--yes-i-confirm-motion", "--max-linear", "0.2", "--max-angular", "1.2"])
		self.assertEqual(cap_speed_limits((0.23, 1.53), raised), (0.2, 1.2))
		self.assertIn("(최고 0.20)", TerminalHoldControl(
			0.6, FakeClock(), *cap_speed_limits((0.23, 1.53), raised)).status())
		# 베이스 상한이 더 낮으면 그것이 이긴다
		self.assertEqual(cap_speed_limits((0.1, 0.5), raised), (0.1, 0.5))
		for bad in ("0", "nan", "-0.1"):
			with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
				build_parser().parse_args(["--yes-i-confirm-motion", "--max-linear", bad])


class TestTerminalHoldControl(unittest.TestCase):
	def setUp(self):
		self.clock = FakeClock()
		self.control = TerminalHoldControl(0.6, self.clock)

	def at(self, seconds):
		self.clock.now = 100.0 + seconds

	def test_key_stays_held_through_keyboard_repeat_delay(self):
		self.control.handle_key("w")
		self.at(0.55)
		self.assertFalse(self.control.expire())
		self.assertEqual(self.control.profile.active_key, "w")
		self.control.handle_key("w")   # 0.5 s 뒤 반복 시작
		self.at(1.0)
		self.assertFalse(self.control.expire())
		self.assertEqual(self.control.profile.active_key, "w")

	def test_key_released_when_repeat_stops(self):
		self.control.handle_key("w")
		self.at(0.61)
		self.assertTrue(self.control.expire())
		self.assertIsNone(self.control.profile.active_key)
		self.assertEqual(self.control.profile.next_motion(), Motion())

	def test_uppercase_counts_as_same_key(self):
		self.control.handle_key("W")
		self.assertEqual(self.control.profile.active_key, "w")

	def test_space_stops_and_ignores_held_key_until_released(self):
		self.control.handle_key("w")
		self.at(0.1)
		self.assertTrue(self.control.handle_key(" "))
		self.assertIsNone(self.control.profile.active_key)
		for seconds in (0.5, 0.55, 0.6):   # 아직 누르고 있다 (반복 입력)
			self.at(seconds)
			self.control.handle_key("w")
			self.control.expire()
			self.assertIsNone(self.control.profile.active_key)
		self.at(1.3)                       # 손을 뗐다
		self.control.expire()
		self.at(1.4)
		self.control.handle_key("w")       # 다시 누름
		self.assertEqual(self.control.profile.active_key, "w")

	def test_switching_direction_publishes_zero_once(self):
		self.control.handle_key("w")
		self.assertGreater(self.control.profile.next_motion().linear_x, 0.0)
		self.control.handle_key("a")
		self.assertEqual(self.control.profile.next_motion(), Motion())
		self.assertGreater(self.control.profile.next_motion().linear_y, 0.0)

	def test_speed_keys_select_stage(self):
		self.control.handle_key("3")
		self.assertEqual(self.control.profile.speed_stage, 3)
		self.control.handle_key("1")
		self.assertEqual(self.control.profile.speed_stage, 1)

	def test_r_and_f_step_speed_one_level(self):
		self.assertEqual(self.control.profile.current_speeds(), (0.04, 0.23))
		self.control.handle_key("r")
		self.assertEqual(self.control.profile.current_speeds(), (0.05, 0.29))
		self.at(0.3)
		self.control.handle_key("f")
		self.at(0.6)
		self.control.handle_key("f")
		self.assertEqual(self.control.profile.current_speeds(), (0.03, 0.17))

	def test_held_r_steps_gradually_not_every_repeat(self):
		for index in range(10):                    # 0.5 s 동안 0.05 s마다 반복 입력
			self.at(index * 0.05)
			self.control.handle_key("r")
		self.assertEqual(self.control.profile.current_speeds()[0], 0.07)   # 0, 0.2, 0.4 s에 한 칸씩

	def press_r_and_f(self, control, count):
		for index in range(count):
			self.at(index * 0.2)
			control.handle_key("r")
		top = control.profile.current_speeds()
		for index in range(count, 2 * count):
			self.at(index * 0.2)
			control.handle_key("f")
		return top, control.profile.current_speeds()

	def test_twenty_levels_without_base_limit(self):
		self.assertEqual(len(self.control.profile.LEVELS), 20)
		top, bottom = self.press_r_and_f(self.control, 30)
		self.assertEqual(top, (0.20, 1.16))
		self.assertEqual(bottom, (0.01, 0.06))

	def test_levels_capped_by_base_controller_limits(self):
		# 예전 베이스 설정 (0.11 · 0.1 · 0.11 m/s 중 최소, 0.75 rad/s) → 0.10 m/s 단계까지
		capped = TerminalHoldControl(0.6, self.clock, max_linear=0.1, max_angular=0.75)
		top, _ = self.press_r_and_f(capped, 30)
		self.assertEqual(top, (0.10, 0.58))
		# 상한을 0.20 m/s · 1.2 rad/s로 올리면 20단계를 모두 쓴다
		raised = TerminalHoldControl(0.6, self.clock, max_linear=0.2, max_angular=1.2)
		top, _ = self.press_r_and_f(raised, 30)
		self.assertEqual(top, (0.20, 1.16))
		# 회전 상한이 먼저 걸리면 그 단계까지만
		yaw_capped = TerminalHoldControl(0.6, self.clock, max_linear=0.2, max_angular=0.75)
		top, _ = self.press_r_and_f(yaw_capped, 30)
		self.assertEqual(top, (0.13, 0.75))

	def test_limits_below_stage_three_are_refused(self):
		with self.assertRaises(ValueError):
			TerminalHoldControl(0.6, self.clock, max_linear=0.05, max_angular=0.75)

	def test_presets_match_window_stages(self):
		for key, speeds in (("1", (0.04, 0.23)), ("2", (0.05, 0.29)), ("3", (0.06, 0.35))):
			self.control.handle_key(key)
			self.assertEqual(self.control.profile.current_speeds(), speeds)

	def test_commanded_speed_follows_level(self):
		self.control.handle_key("r")               # 0.05 m/s
		self.control.handle_key("w")
		motion = Motion()
		for _ in range(30):                         # 가감속(0.005 m/s 씩) 끝까지
			motion = self.control.profile.next_motion()
		self.assertAlmostEqual(motion.linear_x, 0.05)

	def test_status_shows_speed_and_cap(self):
		capped = TerminalHoldControl(0.6, self.clock, max_linear=0.1, max_angular=0.75)
		self.assertIn("0.04 m/s (최고 0.10)", capped.status())

	def test_unknown_keys_do_nothing(self):
		self.assertFalse(self.control.handle_key("x"))
		self.assertIsNone(self.control.profile.active_key)


class TestTerminalSession(unittest.TestCase):
	def setUp(self):
		self.clock = FakeClock()
		self.endpoint = FakeEndpoint()
		self.config = GuardedTeleopConfig(zero_cycles=3)

	def run_session(self, steps):
		return run_terminal_session(
			self.endpoint, KeyScript(self.clock, steps), config=self.config, clock=self.clock)

	def test_arms_drives_while_held_stops_after_release_and_disarms(self):
		result = self.run_session(held("w", 1.0) + idle(1.0) + [(0.05, KEY_EXIT)])
		self.assertEqual(result, "exit")
		events = self.endpoint.events
		self.assertEqual(events[:3], [("publish", Motion())] * 3)
		self.assertEqual(events[3], ("power", True))
		self.assertEqual(events[-4:], [("publish", Motion())] * 3 + [("power", False)])
		motions = self.endpoint.publishes()[3:-3]
		self.assertTrue(any(motion.linear_x > 0.0 for motion in motions))
		self.assertTrue(all(motion.linear_y == 0.0 and motion.angular_z == 0.0 for motion in motions))
		# 손을 떼고 0.6 s 남짓 지나면 멈춰 있다
		self.assertEqual(motions[-1], Motion())

	def test_tap_moves_briefly_then_stops(self):
		self.run_session([(0.05, "d")] + idle(1.0))
		motions = self.endpoint.publishes()[3:-3]
		self.assertTrue(any(motion.linear_y < 0.0 for motion in motions))
		self.assertEqual(motions[-1], Motion())

	def test_ready_failure_prevents_motion(self):
		self.endpoint.ready = False
		with self.assertRaises(RuntimeError):
			self.run_session(held("w", 0.5))
		self.assertTrue(all(motion == Motion() for motion in self.endpoint.publishes()))
		self.assertEqual(self.endpoint.events[-1], ("power", False))

	def test_ready_false_during_session_stops_and_disarms(self):
		self.endpoint.ready_after_publishes = 10
		with self.assertRaises(RuntimeError):
			self.run_session(held("w", 2.0))
		self.assertEqual(self.endpoint.events[-1], ("power", False))

	def test_ctrl_c_cleans_up(self):
		with self.assertRaises(KeyboardInterrupt):
			self.run_session(held("q", 0.3) + [(0.05, KEY_CTRL_C)])
		self.assertEqual(self.endpoint.events[-1], ("power", False))

	def test_session_timeout_cleans_up(self):
		self.config = GuardedTeleopConfig(zero_cycles=3, max_session_duration=1.0)
		result = self.run_session(held("w", 3.0))
		self.assertEqual(result, "timeout")
		self.assertEqual(self.endpoint.events[-1], ("power", False))

	def test_escape_sequences_are_ignored(self):
		result = self.run_session([(0.05, KEY_IGNORED)] * 5 + [(0.05, KEY_EXIT)])
		self.assertEqual(result, "exit")
		self.assertTrue(all(motion == Motion() for motion in self.endpoint.publishes()))


class TestTerminalKeyInput(unittest.TestCase):
	def setUp(self):
		read_fd, self.write_fd = os.pipe()
		self.stream = os.fdopen(read_fd, "rb", buffering=0)
		self.input = TerminalKeyInput(self.stream)

	def tearDown(self):
		self.stream.close()
		os.close(self.write_fd)

	def test_plain_key(self):
		os.write(self.write_fd, b"w")
		self.assertEqual(self.input.read_key(0.1), "w")

	def test_no_key_returns_none(self):
		self.assertIsNone(self.input.read_key(0.01))

	def test_escape_alone_exits(self):
		os.write(self.write_fd, b"\x1b")
		self.assertEqual(self.input.read_key(0.1), KEY_EXIT)

	def test_arrow_key_sequence_is_ignored(self):
		os.write(self.write_fd, b"\x1b[A")
		self.assertEqual(self.input.read_key(0.1), KEY_IGNORED)
		self.assertIsNone(self.input.read_key(0.01))

	def test_ctrl_c_byte_is_returned(self):
		os.write(self.write_fd, b"\x03")
		self.assertEqual(self.input.read_key(0.1), KEY_CTRL_C)


if __name__ == "__main__":
	unittest.main()
