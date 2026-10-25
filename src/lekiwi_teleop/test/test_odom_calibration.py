import argparse
import contextlib
import io
import unittest

from lekiwi_teleop.odom_calibration import (
	CalibrationSample,
	build_parser,
	calculate_scale_recommendation,
	finite_non_zero_float,
	finite_positive_float,
	main,
)


class TestOdomCalibration(unittest.TestCase):
	def test_calculate_scale_recommendation_scales_current_calibration(self):
		sample = CalibrationSample(
			current_scale=1.0,
			odom_delta=0.6,
			measured_delta=0.5,
		)

		recommendation = calculate_scale_recommendation(sample)

		self.assertAlmostEqual(recommendation.recommended_scale, 5.0 / 6.0)
		self.assertAlmostEqual(recommendation.odom_to_measured_ratio, 1.2)

	def test_calculate_scale_recommendation_uses_absolute_odom_delta_for_reverse_motion(self):
		sample = CalibrationSample(
			current_scale=1.0,
			odom_delta=-0.48,
			measured_delta=0.5,
		)

		recommendation = calculate_scale_recommendation(sample)

		self.assertAlmostEqual(recommendation.recommended_scale, 0.5 / 0.48)
		self.assertAlmostEqual(recommendation.odom_to_measured_ratio, 0.96)

	def test_calculate_scale_recommendation_rejects_invalid_inputs(self):
		with self.assertRaisesRegex(ValueError, "current_scale"):
			calculate_scale_recommendation(
				CalibrationSample(
					current_scale=0.0,
					odom_delta=0.5,
					measured_delta=0.5,
				)
			)

		with self.assertRaisesRegex(ValueError, "odom_delta"):
			calculate_scale_recommendation(
				CalibrationSample(
					current_scale=1.0,
					odom_delta=0.0,
					measured_delta=0.5,
				)
			)

		with self.assertRaisesRegex(ValueError, "measured_delta"):
			calculate_scale_recommendation(
				CalibrationSample(
					current_scale=1.0,
					odom_delta=0.5,
					measured_delta=0.0,
				)
			)

	def test_parser_feeds_calculation_contract(self):
		parser = build_parser()
		args = parser.parse_args(
			[
				"--current-scale",
				"1.0",
				"--odom-delta",
				"0.45",
				"--measured-delta",
				"0.5",
			]
		)

		recommendation = calculate_scale_recommendation(CalibrationSample.from_args(args))

		self.assertAlmostEqual(recommendation.recommended_scale, 0.5 / 0.45)

	def test_main_prints_recommended_launch_argument(self):
		stdout = io.StringIO()

		with contextlib.redirect_stdout(stdout):
			exit_code = main(
				[
					"--current-scale",
					"1.0",
					"--odom-delta",
					"0.6",
					"--measured-delta",
					"0.5",
				]
			)

		self.assertEqual(exit_code, 0)
		output = stdout.getvalue()
		self.assertIn("recommended encoder_odom_scale: 0.833333", output)
		self.assertIn("launch argument: encoder_odom_scale:=0.833333", output)

	def test_finite_positive_float_rejects_non_positive_or_non_finite_values(self):
		with self.assertRaisesRegex(argparse.ArgumentTypeError, "0보다 큰 유한한 값"):
			finite_positive_float("0")
		with self.assertRaisesRegex(argparse.ArgumentTypeError, "0보다 큰 유한한 값"):
			finite_positive_float("nan")

	def test_finite_non_zero_float_rejects_zero_or_non_finite_values(self):
		self.assertEqual(finite_non_zero_float("-0.5"), -0.5)

		with self.assertRaisesRegex(argparse.ArgumentTypeError, "0이 아닌 유한한 값"):
			finite_non_zero_float("0")
		with self.assertRaisesRegex(argparse.ArgumentTypeError, "0이 아닌 유한한 값"):
			finite_non_zero_float("inf")

	def test_parser_rejects_invalid_odom_delta_before_calculation(self):
		parser = build_parser()

		with contextlib.redirect_stderr(io.StringIO()):
			with self.assertRaises(SystemExit) as zero_context:
				parser.parse_args(
					[
						"--current-scale",
						"1.0",
						"--odom-delta",
						"0",
						"--measured-delta",
						"0.5",
					]
				)

		with contextlib.redirect_stderr(io.StringIO()):
			with self.assertRaises(SystemExit) as nan_context:
				parser.parse_args(
					[
						"--current-scale",
						"1.0",
						"--odom-delta",
						"nan",
						"--measured-delta",
						"0.5",
					]
				)

		self.assertEqual(zero_context.exception.code, 2)
		self.assertEqual(nan_context.exception.code, 2)


if __name__ == "__main__":
	unittest.main()
