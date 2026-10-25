"""Shared ROS motor connection for supervised teleop, calibration and navigation."""

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from lekiwi_teleop.interface_names import (
	CMD_VEL_TOPIC,
	MOTOR_POWER_SERVICE,
	MOTOR_READY_TOPIC,
)


@dataclass(frozen=True)
class Motion:
	linear_x: float = 0.0
	linear_y: float = 0.0
	angular_z: float = 0.0


def request_motor_power(node, client, enabled: bool, *, timeout_sec: float | None = None) -> None:
	"""Require acknowledgement, optionally sharing a total discovery/response budget.

	Without a total budget, preserve safe_cmd_vel's separate 2s discovery and
	3s response limits. Supervised sessions pass their remaining time budget.
	"""
	if timeout_sec is not None and (not math.isfinite(timeout_sec) or timeout_sec <= 0.0):
		raise RuntimeError(f"{MOTOR_POWER_SERVICE} timeout budget is exhausted")
	deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
	discovery_timeout = 2.0 if timeout_sec is None else min(2.0, timeout_sec)
	if not client.wait_for_service(timeout_sec=discovery_timeout):
		raise RuntimeError(f"{MOTOR_POWER_SERVICE} service unavailable after {discovery_timeout:g}s")
	request = SetBool.Request()
	request.data = enabled
	future = client.call_async(request)
	remaining = 3.0 if deadline is None else deadline - time.monotonic()
	if remaining <= 0.0:
		raise RuntimeError(f"{MOTOR_POWER_SERVICE} service deadline expired before response")
	response_timeout = min(3.0, remaining)
	rclpy.spin_until_future_complete(node, future, timeout_sec=response_timeout)
	if not future.done():
		raise RuntimeError(f"{MOTOR_POWER_SERVICE} service response timed out after {response_timeout:g}s")
	response = future.result()
	if response is None or not response.success:
		detail = "" if response is None else response.message
		raise RuntimeError(f"{MOTOR_POWER_SERVICE} {enabled} failed: {detail}")


def cleanup_session(endpoint, zero_cycles: int) -> BaseException | None:
	errors: list[BaseException] = []
	for _ in range(zero_cycles):
		try:
			endpoint.publish_motion(Motion())
		except BaseException as error:
			errors.append(error)
	try:
		endpoint.set_motor_power(False)
	except BaseException as error:
		errors.append(error)

	if not errors:
		return None
	if len(errors) == 1:
		return errors[0]
	return RuntimeError(
		"cleanup failures: " + "; ".join(str(error) for error in errors)
	)


class RosMotorEndpoint:
	def __init__(self, *, publish_commands=True, node_name="lekiwi_guarded_base_teleop"):
		self.publish_commands = publish_commands
		self.node_name = node_name
		self.node = None
		self.publisher = None
		self.motor_power_client = None
		self.ready_subscription = None
		self._latest_ready: bool | None = None

	@property
	def motor_ready(self) -> bool | None:
		return self._latest_ready

	def initialize(self) -> None:
		self.node = rclpy.create_node(self.node_name)
		if self.publish_commands:
			self.publisher = self.node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
		self.motor_power_client = self.node.create_client(SetBool, MOTOR_POWER_SERVICE)
		ready_qos = QoSProfile(
			depth=1,
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
		)
		self.ready_subscription = self.node.create_subscription(
			Bool, MOTOR_READY_TOPIC, self._on_motor_ready, ready_qos
		)

	def _on_motor_ready(self, message: Bool) -> None:
		self._latest_ready = bool(message.data)

	def publish_motion(self, motion: Motion) -> None:
		if self.publisher is None:
			raise RuntimeError(f"this endpoint does not publish {CMD_VEL_TOPIC}")
		message = Twist()
		message.linear.x = motion.linear_x
		message.linear.y = motion.linear_y
		message.angular.z = motion.angular_z
		self.publisher.publish(message)

	def set_motor_power(self, enabled: bool, timeout_sec: float = 5.0) -> None:
		request_motor_power(self.node, self.motor_power_client, enabled, timeout_sec=timeout_sec)

	def begin_ready_observation(self) -> None:
		self._latest_ready = None

	def wait_for_motor_ready(self, timeout_sec: float) -> bool:
		if self._latest_ready is True:
			return True
		deadline = time.monotonic() + timeout_sec
		while rclpy.ok():
			remaining = deadline - time.monotonic()
			if remaining <= 0.0:
				return False
			rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
			if self._latest_ready is True:
				return True
		return False

	def spin_once(self) -> None:
		"""대기 없이 받은 메시지(예: lekiwi/motor/ready)만 처리한다."""
		rclpy.spin_once(self.node, timeout_sec=0.0)

	def report_cleanup_error(self, message: str) -> None:
		self.node.get_logger().error(message)

	def destroy(self) -> None:
		if self.node is not None:
			self.node.destroy_node()
