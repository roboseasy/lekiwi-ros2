"""ROS message validation without starting a ROS context or any hardware."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
from rclpy.parameter import Parameter

from lekiwi_teleop import distance_trial_ros as ros
from lekiwi_teleop.distance_trial import Trial, validate_parameters
from lekiwi_teleop.distance_trial_ros import DistanceEndpoint


class TestDistanceOdometry(unittest.TestCase):
	def setUp(self):
		self.endpoint = DistanceEndpoint()
		self.endpoint.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
			now=lambda: SimpleNamespace(nanoseconds=200_000_000_000)))
		self.endpoint.expected_frames = ("odom", "base_footprint")

	def message(self, seconds=200., x=0., yaw=0.):
		msg = Odometry()
		ns = round(seconds * 1e9)
		msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
		msg.header.frame_id, msg.child_frame_id = "odom", "base_footprint"
		msg.pose.pose.position.x = x
		msg.pose.pose.orientation.z = math.sin(yaw / 2)
		msg.pose.pose.orientation.w = math.cos(yaw / 2)
		return msg

	def test_old_or_nonadvancing_stamps_are_rejected(self):
		self.endpoint.on_odom(self.message(199.))
		self.assertIn("Stale", self.endpoint.error)
		self.setUp()
		self.endpoint.on_odom(self.message())
		self.endpoint.on_odom(self.message())
		self.assertIn("advance", self.endpoint.error)

	def test_position_and_yaw_resets_are_rejected(self):
		for options in ({"x": .1}, {"yaw": .3}):
			self.setUp()
			self.endpoint.on_odom(self.message())
			self.endpoint.on_odom(self.message(200.02, **options))
			self.assertIn("jumped", self.endpoint.error)

	def test_quaternion_and_frames_are_checked(self):
		msg = self.message()
		msg.pose.pose.orientation.w = 0.
		self.endpoint.on_odom(msg)
		self.assertIn("quaternion", self.endpoint.error)
		self.setUp()
		msg = self.message()
		msg.child_frame_id = "different_base"
		self.endpoint.on_odom(msg)
		self.assertIn("frames", self.endpoint.error)

	def test_receive_gap_cannot_be_hidden_by_a_fresh_latest_sample(self):
		with patch("lekiwi_teleop.distance_trial_ros.time.monotonic", side_effect=[10., 10.4]):
			self.endpoint.on_odom(self.message())
			self.endpoint.on_odom(self.message(200.02))
		self.assertIn("gap", self.endpoint.error)

	def test_small_motion_is_accepted_and_recorded(self):
		self.endpoint.on_odom(self.message())
		self.endpoint.on_odom(self.message(200.02, x=.0004, yaw=.001))
		self.assertIsNone(self.endpoint.error)
		self.assertEqual(len(self.endpoint.samples), 2)
		self.assertAlmostEqual(self.endpoint.latest["yaw"], .001)


# 베이스 컨트롤러 기본값 (저장소 base_operation.md 기준)
BASE_VALUES = {
	"motor_backend": "driver", "enable_motor_write": True, "torque_enable": False,
	"odom_source": "encoder", "encoder_feedback_source": "position",
	"use_sim_time": False, "cmd_vel_timeout": .3, "max_linear_x": .11,
	"max_linear_y": .10, "max_linear_speed": .11,
	"max_wheel_speed": 2.3009711818284617, "wheel_radius": .0508,
	"base_radius": .115, "wheel_ids": [8, 9, 7], "wheel_directions": [1., 1., 1.],
	"encoder_odom_scale": 1., "encoder_position_ticks_per_revolution": 4096.,
	"kinematics_frame_x": 0., "kinematics_frame_y": 0., "kinematics_frame_yaw": 0.,
	"odom_frame": "odom", "base_frame": "base_footprint",
}
DRIVER_VALUES = {"speed_tick_scale": 651.8986469044033, "speed_tick_limit": 1500}


class FakeParameterClient:
	"""노드별 파라미터 표를 돌려주는 AsyncParameterClient 대역."""

	tables = {}
	available = {}
	requests = []

	def __init__(self, _node, remote_node):
		self.remote_node = remote_node

	def wait_for_services(self, timeout_sec):
		return self.available.get(self.remote_node, True)

	def get_parameters(self, names):
		self.requests.append((self.remote_node, list(names)))
		table = self.tables.get(self.remote_node, {})
		values = [
			Parameter(name, value=table[name]).get_parameter_value()
			if name in table else ParameterValue()
			for name in names
		]
		result = SimpleNamespace(values=values)
		return SimpleNamespace(done=lambda: True, result=lambda: result)


class TestDistanceParameters(unittest.TestCase):
	def setUp(self):
		self.endpoint = DistanceEndpoint()
		self.endpoint.node = SimpleNamespace(get_namespace=lambda: "/")
		FakeParameterClient.tables = {
			"/lekiwi_base_controller_node": dict(BASE_VALUES),
			"/lekiwi_motor_driver_node": dict(DRIVER_VALUES),
		}
		FakeParameterClient.available = {}
		FakeParameterClient.requests = []

	def read(self):
		with patch.object(ros, "AsyncParameterClient", FakeParameterClient), \
				patch.object(ros.rclpy, "spin_until_future_complete"):
			return self.endpoint.parameters()

	def test_speed_tick_parameters_come_from_the_motor_driver(self):
		result = self.read()
		(base_node, base_names), (driver_node, driver_names) = FakeParameterClient.requests
		self.assertEqual(base_node, "/lekiwi_base_controller_node")
		self.assertEqual(driver_node, "/lekiwi_motor_driver_node")
		self.assertEqual(driver_names, ["speed_tick_scale", "speed_tick_limit"])
		self.assertNotIn("speed_tick_scale", base_names)
		self.assertNotIn("speed_tick_limit", base_names)
		self.assertEqual(result, {**BASE_VALUES, **DRIVER_VALUES})
		self.assertEqual(self.endpoint.expected_frames, ("odom", "base_footprint"))
		validate_parameters(result, Trial(), False)

	def test_speed_tick_parameters_on_the_base_controller_are_not_enough(self):
		FakeParameterClient.tables["/lekiwi_base_controller_node"].update(DRIVER_VALUES)
		FakeParameterClient.tables["/lekiwi_motor_driver_node"] = {}
		with self.assertRaisesRegex(RuntimeError, "Missing /lekiwi_motor_driver_node"):
			self.read()

	def test_unavailable_motor_driver_parameters_stop_the_trial(self):
		FakeParameterClient.available["/lekiwi_motor_driver_node"] = False
		with self.assertRaisesRegex(RuntimeError, "lekiwi_motor_driver_node parameter service"):
			self.read()

	def test_missing_base_parameter_stops_the_trial(self):
		del FakeParameterClient.tables["/lekiwi_base_controller_node"]["wheel_radius"]
		with self.assertRaisesRegex(RuntimeError, "Missing /lekiwi_base_controller_node"):
			self.read()


class GraphNode:
	def __init__(self, publishers, subscribers):
		self.publishers = publishers
		self.subscribers = subscribers

	def get_namespace(self):
		return "/"

	def get_fully_qualified_name(self):
		return "/lekiwi_guarded_base_teleop"

	@staticmethod
	def infos(names):
		return [SimpleNamespace(node_namespace="/", node_name=name) for name in names]

	def get_publishers_info_by_topic(self, topic):
		return self.infos(self.publishers.get(topic, []))

	def get_subscriptions_info_by_topic(self, topic):
		return self.infos(self.subscribers.get(topic, []))


class TestDistanceGraph(unittest.TestCase):
	BASE = "lekiwi_base_controller_node"

	def graph(self, **changes):
		publishers = {
			"cmd_vel": ["lekiwi_guarded_base_teleop"],
			"odom": [self.BASE],
			"lekiwi/motor/ready": [self.BASE],
		}
		subscribers = {"cmd_vel": [self.BASE]}
		publishers.update(changes.get("publishers", {}))
		subscribers.update(changes.get("subscribers", {}))
		endpoint = DistanceEndpoint()
		endpoint.node = GraphNode(publishers, subscribers)
		return endpoint

	def test_base_controller_owns_the_command_path_and_feedback(self):
		self.graph().check_graph()

	def test_other_publishers_or_the_previous_base_name_are_rejected(self):
		for changes in (
			{"subscribers": {"cmd_vel": ["lekiwi_node"]}},
			{"subscribers": {"cmd_vel": []}},
			{"publishers": {"cmd_vel": ["lekiwi_guarded_base_teleop", "collision_monitor"]}},
			{"publishers": {"odom": ["lekiwi_node"]}},
			{"publishers": {"lekiwi/motor/ready": []}},
		):
			with self.subTest(changes=changes), self.assertRaises(RuntimeError):
				self.graph(**changes).check_graph()


if __name__ == "__main__":
	unittest.main()
