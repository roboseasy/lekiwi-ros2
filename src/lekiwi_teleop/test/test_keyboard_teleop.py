"""동시 입력 키보드 조작 시험: 키를 합치는 규칙, 빠른 가속, 속도 단계와 베이스 상한. ROS · 화면 없음."""

import contextlib
import io
import math
import unittest
from unittest import mock

from lekiwi_teleop import keyboard_teleop
from lekiwi_teleop.keyboard_teleop import CombinedKeySpeedProfile
from lekiwi_teleop.motor_session import Motion
from lekiwi_teleop.terminal_teleop import DEFAULT_MAX_ANGULAR, DEFAULT_MAX_LINEAR


class FakeClock:
	def __init__(self):
		self.now = 0.0

	def __call__(self):
		return self.now


def settle(profile, cycles=40):
	motion = Motion()
	for _ in range(cycles):
		motion = profile.next_motion()
	return motion


class TestCombinedKeySpeedProfile(unittest.TestCase):
	def setUp(self):
		self.clock = FakeClock()
		self.profile = CombinedKeySpeedProfile(0.2, 1.2, self.clock)

	def test_starts_at_ten_centimetres_per_second(self):
		self.assertEqual(self.profile.current_speeds(), (0.10, 0.58))
		self.assertEqual(self.profile.next_motion(), Motion())

	def test_w_plus_d_moves_diagonally_at_level_speed(self):
		self.profile.press("w")
		self.profile.press("d")
		motion = settle(self.profile)
		self.assertAlmostEqual(motion.linear_x, 0.10 / math.sqrt(2.0))
		self.assertAlmostEqual(motion.linear_y, -0.10 / math.sqrt(2.0))
		self.assertAlmostEqual(math.hypot(motion.linear_x, motion.linear_y), 0.10)
		self.assertEqual(self.profile.active_key, "w+d")

	def test_translation_and_rotation_combine(self):
		self.profile.press("w")
		self.profile.press("q")
		motion = settle(self.profile)
		self.assertAlmostEqual(motion.linear_x, 0.10)
		self.assertAlmostEqual(motion.angular_z, 0.58)

	def test_opposite_keys_cancel_that_axis(self):
		self.profile.press("w")
		self.profile.press("s")
		self.profile.press("a")
		motion = settle(self.profile)
		self.assertAlmostEqual(motion.linear_x, 0.0)
		self.assertAlmostEqual(motion.linear_y, 0.10)

	def test_all_six_directions_have_expected_signs(self):
		expected = {
			"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0),
			"q": (0, 0, 1), "e": (0, 0, -1)}
		for key, signs in expected.items():
			profile = CombinedKeySpeedProfile(0.2, 1.2, self.clock)
			profile.press(key)
			motion = settle(profile)
			values = (motion.linear_x, motion.linear_y, motion.angular_z)
			for value, sign in zip(values, signs):
				self.assertEqual(math.copysign(1, value) if value else 0, sign, key)

	def test_fast_acceleration_reaches_top_speed_in_half_a_second(self):
		for _ in range(10):
			self.clock.now += 0.25                    # r 반복 간격(0.2 s)보다 길게
			self.profile.press("r")
		self.assertEqual(self.profile.current_speeds()[0], 0.20)
		self.profile.press("w")
		speeds = [self.profile.next_motion().linear_x for _ in range(10)]   # 10 × 50 ms
		self.assertAlmostEqual(speeds[0], 0.02)
		self.assertAlmostEqual(speeds[-1], 0.20)

	def test_releasing_one_key_keeps_the_other(self):
		self.profile.press("w")
		self.profile.press("d")
		settle(self.profile)
		self.assertFalse(self.profile.release("d"))
		motion = settle(self.profile)
		self.assertAlmostEqual(motion.linear_x, 0.10)
		self.assertAlmostEqual(motion.linear_y, 0.0)

	def test_releasing_all_keys_stops_immediately(self):
		self.profile.press("w")
		settle(self.profile)
		self.assertTrue(self.profile.release("w"))
		self.assertEqual(self.profile.control, Motion())
		self.assertEqual(self.profile.next_motion(), Motion())

	def test_repeated_press_of_held_key_is_ignored(self):
		self.assertTrue(self.profile.press("w"))
		self.assertFalse(self.profile.press("w"))
		self.assertEqual(self.profile.held_keys, ["w"])

	def test_space_stops_everything(self):
		self.profile.press("w")
		self.profile.press("q")
		settle(self.profile)
		self.profile.press("space")
		self.assertEqual(self.profile.held_keys, [])
		self.assertEqual(self.profile.next_motion(), Motion())

	def test_r_and_f_step_with_rate_limit(self):
		self.profile.press("r")
		self.assertEqual(self.profile.current_speeds()[0], 0.11)
		self.clock.now = 0.1
		self.profile.press("r")                       # 0.2 s 안 반복은 무시
		self.assertEqual(self.profile.current_speeds()[0], 0.11)
		self.clock.now = 0.3
		self.profile.press("f")
		self.assertEqual(self.profile.current_speeds()[0], 0.10)

	def test_number_keys_do_nothing(self):
		for key in ("1", "2", "3"):
			self.assertFalse(self.profile.press(key))
		self.assertEqual(self.profile.current_speeds()[0], 0.10)
		self.assertEqual(self.profile.held_keys, [])

	def test_levels_capped_by_base_limits(self):
		capped = CombinedKeySpeedProfile(0.1, 0.75, self.clock)
		self.assertEqual(capped.current_speeds(), (0.10, 0.58))
		for index in range(30):
			self.clock.now += 0.2
			capped.press("r")
		self.assertEqual(capped.current_speeds(), (0.10, 0.58))

	def test_start_level_respects_low_limit(self):
		low = CombinedKeySpeedProfile(0.06, 0.75, self.clock)
		self.assertEqual(low.current_speeds(), (0.06, 0.35))

	def test_limits_too_low_are_refused(self):
		with self.assertRaises(ValueError):
			CombinedKeySpeedProfile(0.03, 0.75, self.clock)

	def test_default_tool_cap_keeps_the_old_top_speed(self):
		# 베이스 상한 0.23 m/s · 1.53 rad/s (2026-10-22) 아래에서도 기본 최고는 예전과 같은 0.10 m/s 단계다
		profile = CombinedKeySpeedProfile(
			min(0.23, DEFAULT_MAX_LINEAR), min(1.53, DEFAULT_MAX_ANGULAR), self.clock)
		self.assertEqual(profile.LEVELS[profile.max_level], (0.10, 0.58))

	def test_main_accepts_speed_cap_options(self):
		# 옵션 값이 틀리면 ROS · 창을 열기 전에 끝난다
		with mock.patch.object(keyboard_teleop.rclpy, "init") as init:
			for bad in ("0", "nan"):
				with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
					keyboard_teleop.main(["--yes-i-confirm-motion", "--max-linear", bad])
				with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
					keyboard_teleop.main(["--yes-i-confirm-motion", "--max-angular", bad])
		init.assert_not_called()

	def test_status_text(self):
		self.profile.press("w")
		self.profile.press("d")
		self.assertIn("w+d", self.profile.status_text())
		self.assertIn("(최고 0.20)", self.profile.status_text())


if __name__ == "__main__":
	unittest.main()
