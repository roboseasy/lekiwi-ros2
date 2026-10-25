"""Keep wheel power enabled only during a supervised Nav2 session."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

from lekiwi_teleop.interface_names import (
	BASE_CONTROLLER_NODE,
	CMD_VEL_TOPIC,
	DRIVER_MOTOR_BACKEND,
	MOTOR_READY_TOPIC,
	NAV2_VELOCITY_NODE,
	ODOM_TOPIC,
	SCAN_FILTERED_TOPIC,
	qualified_node_name,
)
from lekiwi_teleop.motor_session import RosMotorEndpoint


def positive_duration(value: str) -> float:
	duration = float(value)
	if not math.isfinite(duration) or duration <= 0:
		raise argparse.ArgumentTypeError("a finite, positive duration is required")
	return duration


def operator_node_names(value: str) -> tuple[str, ...]:
	operator_names = tuple(name.strip().lstrip("/") for name in value.split(",") if name.strip())
	if not operator_names:
		raise argparse.ArgumentTypeError("at least one operator node name is required")
	return operator_names


def operator_present(node, operator_names) -> bool:
	"""True when no operator view is required or one of the named nodes is in the ROS graph.

	The operator's RViz may run on another machine, so its node in the graph stands for
	"someone is watching". Only the node name is compared, in any namespace.
	"""
	if not operator_names:
		return True
	return any(name in operator_names for name, _namespace in node.get_node_names_and_namespaces())


def names(endpoints) -> set[str]:
	return {
		f"{info.node_namespace.rstrip('/')}/{info.node_name}"
		for info in endpoints
	}


def check_command_graph(node) -> None:
	namespace = node.get_namespace()
	nav2_node = qualified_node_name(namespace, NAV2_VELOCITY_NODE)
	base_node = qualified_node_name(namespace, BASE_CONTROLLER_NODE)
	publisher_infos = node.get_publishers_info_by_topic(CMD_VEL_TOPIC)
	subscriber_infos = node.get_subscriptions_info_by_topic(CMD_VEL_TOPIC)
	publishers = names(publisher_infos)
	subscribers = names(subscriber_infos)
	if len(publisher_infos) != 1 or publishers != {nav2_node}:
		raise RuntimeError(
			f"expected only {nav2_node} on {CMD_VEL_TOPIC}; found {sorted(publishers)}"
		)
	if len(subscriber_infos) != 1 or subscribers != {base_node}:
		raise RuntimeError(
			f"expected only {base_node} on {CMD_VEL_TOPIC}; found {sorted(subscribers)}"
		)


def check_hardware_parameters(values) -> None:
	backend, writes, odom, initial_torque = values
	if backend.string_value != DRIVER_MOTOR_BACKEND or not writes.bool_value:
		raise RuntimeError(
			f"base is not using the writable '{DRIVER_MOTOR_BACKEND}' motor backend"
		)
	if odom.string_value != "encoder":
		raise RuntimeError("Nav2 requires encoder odometry on the physical base")
	if initial_torque.bool_value:
		raise RuntimeError("base started with torque enabled; start it with torque_enable:=false")


def read_hardware_parameters(node) -> None:
	base_node = qualified_node_name(node.get_namespace(), BASE_CONTROLLER_NODE)
	client = AsyncParameterClient(node, base_node)
	if not client.wait_for_services(timeout_sec=3.0):
		raise RuntimeError(f"{base_node} parameter services are unavailable")
	future = client.get_parameters([
		"motor_backend", "enable_motor_write", "odom_source", "torque_enable",
	])
	rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
	if not future.done() or future.result() is None:
		raise RuntimeError("could not read base hardware parameters")
	check_hardware_parameters(future.result().values)


class SensorObserver:
	def __init__(self, node, clock=time.monotonic):
		self.clock = clock
		self.scan_time = None
		self.odom_time = None
		self.scan_subscription = node.create_subscription(
			LaserScan, SCAN_FILTERED_TOPIC, self._on_scan, qos_profile_sensor_data
		)
		self.odom_subscription = node.create_subscription(
			Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data
		)

	def _on_scan(self, _message):
		self.scan_time = self.clock()

	def _on_odom(self, _message):
		self.odom_time = self.clock()

	def fresh(self, max_age=1.0):
		now = self.clock()
		return (
			self.scan_time is not None
			and self.odom_time is not None
			and now - self.scan_time <= max_age
			and now - self.odom_time <= max_age
		)


def publish_zero(node) -> None:
	publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
	try:
		for _ in range(20):
			publisher.publish(Twist())
			rclpy.spin_once(node, timeout_sec=0.01)
	finally:
		node.destroy_publisher(publisher)


def supervise(
	endpoint, duration, startup_timeout=5.0, clock=time.monotonic, operator_names=(),
) -> None:
	node = endpoint.node
	observer = SensorObserver(node, clock)
	deadline = clock() + startup_timeout
	graph_error = None
	while clock() < deadline and rclpy.ok():
		rclpy.spin_once(node, timeout_sec=0.1)
		if not observer.fresh() or endpoint.motor_ready is None:
			continue
		if endpoint.motor_ready is True:
			raise RuntimeError(
				f"{MOTOR_READY_TOPIC} must be false before this guard arms the motors"
			)
		if not operator_present(node, operator_names):
			continue
		try:
			check_command_graph(node)
			break
		except RuntimeError as error:
			graph_error = error
	else:
		if graph_error is not None and observer.fresh():
			raise graph_error
	if not observer.fresh():
		raise RuntimeError(
			f"fresh {SCAN_FILTERED_TOPIC} and {ODOM_TOPIC} were not observed"
		)
	if endpoint.motor_ready is not False:
		raise RuntimeError(
			f"{MOTOR_READY_TOPIC} must be false before this guard arms the motors"
		)
	if not operator_present(node, operator_names):
		raise RuntimeError(
			f"no operator view ({', '.join(operator_names)}) is running; "
			"open RViz before Nav2 can move the base"
		)
	check_command_graph(node)
	read_hardware_parameters(node)
	publish_zero(node)
	graph_deadline = clock() + 2.0
	while True:
		try:
			check_command_graph(node)
			break
		except RuntimeError:
			if clock() >= graph_deadline:
				raise
			rclpy.spin_once(node, timeout_sec=0.05)
	if not observer.fresh():
		raise RuntimeError(
			f"{SCAN_FILTERED_TOPIC} or {ODOM_TOPIC} became stale before motor power on"
		)
	if not operator_present(node, operator_names):
		raise RuntimeError("operator view closed before motor power on")

	endpoint.begin_ready_observation()
	endpoint.set_motor_power(True)
	if not endpoint.wait_for_motor_ready(5.0):
		raise RuntimeError(
			f"{MOTOR_READY_TOPIC} did not become true after motor power on"
		)
	print(
		"Nav2 motor guard active. Completed goals keep motor power on; "
		"close RViz or stop the launch to disarm.",
		flush=True,
	)

	end = clock() + duration
	next_graph_check = clock()
	while rclpy.ok() and clock() < end:
		rclpy.spin_once(node, timeout_sec=0.1)
		if endpoint.motor_ready is not True:
			raise RuntimeError(
				f"{MOTOR_READY_TOPIC} became false; Nav2 motor guard is disarming"
			)
		if not observer.fresh():
			raise RuntimeError(
				f"{SCAN_FILTERED_TOPIC} or {ODOM_TOPIC} stopped; "
				"Nav2 motor guard is disarming"
			)
		if clock() >= next_graph_check:
			check_command_graph(node)
			if not operator_present(node, operator_names):
				raise RuntimeError("operator view closed; Nav2 motor guard is disarming")
			next_graph_check = clock() + 0.5
	if clock() >= end:
		print("Nav2 motor guard time limit reached; disarming motors.", flush=True)


def main(argv=None) -> int:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--yes-i-confirm-motion", action="store_true", required=True)
	parser.add_argument("--max-session-duration", type=positive_duration, default=600.0)
	parser.add_argument("--startup-timeout", type=positive_duration, default=5.0)
	parser.add_argument(
		"--operator-nodes", type=operator_node_names, default=("rviz", "rviz2"),
		help="comma-separated operator view node names (default rviz,rviz2): arm only while "
		"one is in the ROS graph and disarm when all are gone. The guard does not re-arm; "
		"restart it after the view closes",
	)
	process_args = sys.argv if argv is None else [sys.argv[0], *argv]
	args = parser.parse_args(remove_ros_args(args=process_args)[1:])
	endpoint = RosMotorEndpoint(
		publish_commands=False, node_name="lekiwi_nav2_motor_guard"
	)
	rclpy.init(args=process_args, signal_handler_options=SignalHandlerOptions.NO)
	result = 0
	try:
		endpoint.initialize()
		supervise(
			endpoint, args.max_session_duration, args.startup_timeout,
			operator_names=args.operator_nodes,
		)
	except KeyboardInterrupt:
		result = 130
	except Exception as error:
		print(f"Nav2 motor guard stopped: {error}", file=sys.stderr)
		result = 1
	finally:
		if endpoint.node is not None:
			try:
				endpoint.set_motor_power(False)
			except Exception as error:
				print(f"Motor power off was not acknowledged: {error}", file=sys.stderr)
				result = 1
			try:
				publish_zero(endpoint.node)
			except Exception as error:
				print(f"Zero velocity publish failed: {error}", file=sys.stderr)
				result = 1
			endpoint.destroy()
		rclpy.try_shutdown()
	return result


if __name__ == "__main__":
	raise SystemExit(main())
