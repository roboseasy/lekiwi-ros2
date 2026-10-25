from types import SimpleNamespace
from unittest.mock import patch

import pytest

from lekiwi_teleop import nav2_motor_guard as guard


BASE = "lekiwi_base_controller_node"


def endpoint(name, namespace="/"):
	return SimpleNamespace(node_namespace=namespace, node_name=name)


class GraphNode:
	def __init__(self, publishers, subscribers, namespace="/"):
		self.publishers = publishers
		self.subscribers = subscribers
		self.namespace = namespace

	def get_namespace(self):
		return self.namespace

	def get_publishers_info_by_topic(self, topic):
		assert topic == "cmd_vel"
		return [endpoint(name, self.namespace) for name in self.publishers]

	def get_subscriptions_info_by_topic(self, topic):
		assert topic == "cmd_vel"
		return [endpoint(name, self.namespace) for name in self.subscribers]


def test_nav2_guard_rejects_other_velocity_publishers_and_missing_base():
	guard.check_command_graph(GraphNode(["collision_monitor"], [BASE]))
	with pytest.raises(RuntimeError, match="only /collision_monitor"):
		guard.check_command_graph(GraphNode(
			["collision_monitor", "lekiwi_guarded_base_teleop"], [BASE]
		))
	with pytest.raises(RuntimeError, match="only /collision_monitor"):
		guard.check_command_graph(GraphNode(
			["collision_monitor", "collision_monitor"], [BASE]
		))
	with pytest.raises(RuntimeError, match="only /lekiwi_base_controller_node"):
		guard.check_command_graph(GraphNode(["collision_monitor"], []))


def test_nav2_guard_rejects_the_previous_base_node_name():
	with pytest.raises(RuntimeError, match="only /lekiwi_base_controller_node"):
		guard.check_command_graph(GraphNode(["collision_monitor"], ["lekiwi_node"]))


def test_nav2_guard_graph_names_follow_the_guard_namespace():
	guard.check_command_graph(
		GraphNode(["collision_monitor"], [BASE], namespace="/robot1")
	)

	class MixedNode(GraphNode):
		def get_subscriptions_info_by_topic(self, topic):
			assert topic == "cmd_vel"
			return [endpoint(BASE, "/")]

	with pytest.raises(RuntimeError, match="only /robot1/lekiwi_base_controller_node"):
		guard.check_command_graph(
			MixedNode(["collision_monitor"], [], namespace="/robot1")
		)


def test_nav2_guard_requires_real_encoder_base_with_initial_torque_off():
	def values(backend="driver", writes=True, odom="encoder", torque=False):
		return [
			SimpleNamespace(string_value=backend),
			SimpleNamespace(bool_value=writes),
			SimpleNamespace(string_value=odom),
			SimpleNamespace(bool_value=torque),
		]

	guard.check_hardware_parameters(values())
	for invalid in (
		values(backend="mock"), values(backend="feetech"), values(writes=False),
		values(odom="command"), values(torque=True),
	):
		with pytest.raises(RuntimeError):
			guard.check_hardware_parameters(invalid)


def test_nav2_guard_requires_recent_scan_and_odom():
	class Node:
		def create_subscription(self, _type, topic, callback, _qos):
			self.callbacks[topic] = callback
			return object()

		callbacks = {}

	current = [0.0]
	observer = guard.SensorObserver(Node(), clock=lambda: current[0])
	assert not observer.fresh()
	assert set(Node.callbacks) == {"lekiwi/sensor/scan_filtered", "odom"}
	Node.callbacks["lekiwi/sensor/scan_filtered"](object())
	Node.callbacks["odom"](object())
	assert observer.fresh()
	current[0] = 1.1
	assert not observer.fresh()


def test_nav2_guard_reads_hardware_parameters_from_the_base_controller():
	calls = []

	class Future:
		def done(self):
			return True

		def result(self):
			return SimpleNamespace(values=[
				SimpleNamespace(string_value="driver"),
				SimpleNamespace(bool_value=True),
				SimpleNamespace(string_value="encoder"),
				SimpleNamespace(bool_value=False),
			])

	class Client:
		def __init__(self, _node, remote_node):
			calls.append(("client", remote_node))

		def wait_for_services(self, timeout_sec):
			return True

		def get_parameters(self, names):
			calls.append(("names", list(names)))
			return Future()

	node = SimpleNamespace(get_namespace=lambda: "/")
	with patch.object(guard, "AsyncParameterClient", Client), \
			patch.object(guard.rclpy, "spin_until_future_complete"):
		guard.read_hardware_parameters(node)
	assert calls == [
		("client", "/lekiwi_base_controller_node"),
		("names", ["motor_backend", "enable_motor_write", "odom_source", "torque_enable"]),
	]


def test_nav2_guard_disarms_even_when_preflight_fails():
	class FakeEndpoint:
		last = None

		def __init__(self, **_kwargs):
			FakeEndpoint.last = self
			self.node = None
			self.power_calls = []

		def initialize(self):
			self.node = object()

		def set_motor_power(self, enabled):
			self.power_calls.append(enabled)

		def destroy(self):
			pass

	with patch.object(guard, "RosMotorEndpoint", FakeEndpoint), \
			patch.object(guard.rclpy, "init"), \
			patch.object(guard.rclpy, "try_shutdown"), \
			patch.object(guard, "publish_zero"), \
			patch.object(guard, "supervise", side_effect=RuntimeError("preflight")):
		result = guard.main(["--yes-i-confirm-motion"])
		assert result == 1
		assert FakeEndpoint.last.power_calls == [False]


def test_nav2_guard_waits_for_navigation_command_path_before_power_on():
	now = [0.0]

	class Node(GraphNode):
		def __init__(self):
			super().__init__([], [BASE])

		def get_publishers_info_by_topic(self, topic):
			self.publishers = ["collision_monitor"] if now[0] >= 0.5 else []
			return super().get_publishers_info_by_topic(topic)

	class Endpoint:
		def __init__(self):
			self.node = Node()
			self.motor_ready = False
			self.power_on_at = None

		def begin_ready_observation(self):
			pass

		def set_motor_power(self, enabled):
			assert enabled is True
			self.power_on_at = now[0]
			self.motor_ready = True

		def wait_for_motor_ready(self, _timeout):
			return True

	endpoint = Endpoint()

	def spin(_node, timeout_sec):
		now[0] += timeout_sec

	with patch.object(guard, "SensorObserver") as observer, \
			patch.object(guard.rclpy, "ok", return_value=True), \
			patch.object(guard.rclpy, "spin_once", side_effect=spin), \
			patch.object(guard, "read_hardware_parameters"), \
			patch.object(guard, "publish_zero"):
		observer.return_value.fresh.return_value = True
		guard.supervise(endpoint, duration=0.1, startup_timeout=1.0, clock=lambda: now[0])

	assert endpoint.power_on_at is not None
	assert endpoint.power_on_at >= 0.5


class OperatorGraphNode(GraphNode):
	"""cmd_vel 경로는 갖춰져 있고, operator(now) 가 True일 때만 RViz 노드가 그래프에 있다."""

	def __init__(self, operator):
		super().__init__(["collision_monitor"], [BASE])
		self.operator = operator

	def get_node_names_and_namespaces(self):
		nodes = [("collision_monitor", "/"), (BASE, "/")]
		if self.operator():
			nodes.append(("rviz", "/"))
		return nodes


class OperatorEndpoint:
	def __init__(self, node, now, motor_ready=False):
		self.node = node
		self.now = now
		self.motor_ready = motor_ready
		self.power_calls = []

	def begin_ready_observation(self):
		pass

	def set_motor_power(self, enabled):
		self.power_calls.append((enabled, self.now[0]))
		self.motor_ready = enabled

	def wait_for_motor_ready(self, _timeout):
		return True


def run_supervise_with_operator(operator, duration=0.1, startup_timeout=1.0, motor_ready=False):
	"""감독을 돌리고 (endpoint, 멈춘 이유, 멈춘 시각)을 돌려준다. 정상 종료면 이유는 None."""
	now = [0.0]
	endpoint = OperatorEndpoint(OperatorGraphNode(lambda: operator(now[0])), now, motor_ready)

	def spin(_node, timeout_sec):
		now[0] += timeout_sec

	error = None
	with patch.object(guard, "SensorObserver") as observer, \
			patch.object(guard.rclpy, "ok", return_value=True), \
			patch.object(guard.rclpy, "spin_once", side_effect=spin), \
			patch.object(guard, "read_hardware_parameters"), \
			patch.object(guard, "publish_zero"):
		observer.return_value.fresh.return_value = True
		try:
			guard.supervise(
				endpoint, duration=duration, startup_timeout=startup_timeout,
				clock=lambda: now[0], operator_names=("rviz", "rviz2"))
		except RuntimeError as raised:
			error = raised
	return endpoint, error, now[0]


def test_nav2_guard_waits_for_operator_view_before_power_on():
	endpoint, error, _ = run_supervise_with_operator(lambda now: now >= 0.5)
	assert error is None
	assert endpoint.power_calls[0][0] is True
	assert endpoint.power_calls[0][1] >= 0.5


def test_nav2_guard_does_not_arm_without_operator_view():
	endpoint, error, _ = run_supervise_with_operator(lambda _now: False)
	assert "open RViz" in str(error)
	assert endpoint.power_calls == []


def test_nav2_guard_disarms_when_operator_view_closes():
	# 바로 켜고, 3 s에 RViz가 닫힌다. 감독 시간(60 s)이 끝나기 전, 다음 0.5 s 확인에서 멈춰야 한다.
	endpoint, error, stopped_at = run_supervise_with_operator(
		lambda now: now < 3.0, duration=60.0, startup_timeout=5.0)
	assert "disarming" in str(error)
	assert endpoint.power_calls == [(True, pytest.approx(0.1))]
	assert 3.0 <= stopped_at <= 3.7


def test_nav2_guard_rejects_armed_base_at_once_even_without_operator_view():
	# 토크가 이미 켜져 있으면 RViz를 기다리지 않고 바로 멈춰 main이 토크를 끄게 한다.
	endpoint, error, stopped_at = run_supervise_with_operator(
		lambda _now: False, startup_timeout=60.0, motor_ready=True)
	assert "must be false" in str(error)
	assert stopped_at < 0.5
	assert endpoint.power_calls == []


def test_nav2_guard_operator_names_ignore_namespace_and_slashes():
	node = SimpleNamespace(get_node_names_and_namespaces=lambda: [("rviz2", "/pc")])
	assert guard.operator_present(node, guard.operator_node_names(" /rviz , rviz2 "))
	assert not guard.operator_present(node, ("rviz",))
	assert guard.operator_present(node, ())


def test_nav2_guard_passes_operator_nodes_from_the_command_line():
	motor = SimpleNamespace(node=object())
	motor.initialize = lambda: None
	motor.destroy = lambda: None
	motor.set_motor_power = lambda enabled: None
	with patch.object(guard, 'RosMotorEndpoint', return_value=motor), \
			patch.object(guard.rclpy, 'init'), \
			patch.object(guard.rclpy, 'try_shutdown'), \
			patch.object(guard, 'publish_zero'), \
			patch.object(guard, 'supervise') as supervise:
		assert guard.main(['--yes-i-confirm-motion', '--operator-nodes', 'rviz,rviz2']) == 0
		supervise.assert_called_once_with(motor, 600.0, 5.0, operator_names=('rviz', 'rviz2'))


@pytest.mark.parametrize('use_process_argv', [False, True])
def test_nav2_guard_accepts_ros_arguments_added_by_launch(use_process_argv):
	command = [
		'--yes-i-confirm-motion', '--startup-timeout', '60',
		'--max-session-duration', '3600', '--ros-args', '-r',
		'__node:=lekiwi_nav2_motor_guard',
	]
	process_args = ['lekiwi_nav2_motor_guard', *command]
	motor = SimpleNamespace(node=object())
	motor.initialize = lambda: None
	motor.destroy = lambda: None
	motor.set_motor_power = lambda enabled: None
	with patch.object(guard.sys, 'argv', process_args), \
			patch.object(guard, 'RosMotorEndpoint', return_value=motor), \
			patch.object(guard.rclpy, 'init') as initialize, \
			patch.object(guard.rclpy, 'try_shutdown'), \
			patch.object(guard, 'publish_zero'), \
			patch.object(guard, 'supervise') as supervise:
		assert guard.main(None if use_process_argv else command) == 0
		# --operator-nodes를 주지 않아도 조작 화면을 요구한다
		supervise.assert_called_once_with(motor, 3600.0, 60.0, operator_names=('rviz', 'rviz2'))
		initialize.assert_called_once_with(
			args=process_args,
			signal_handler_options=guard.SignalHandlerOptions.NO,
		)


@pytest.mark.parametrize('command', [
	['--ros-args', '-r', '__node:=lekiwi_nav2_motor_guard'],
	['--yes-i-confirm-motion', '--unknown-option', '--ros-args', '-r', '__node:=guard'],
	['--yes-i-confirm-motion', '--startup-timeout', '-1', '--ros-args', '-r', '__node:=guard'],
	['--yes-i-confirm-motion', '--operator-nodes', ' , ', '--ros-args', '-r', '__node:=guard'],
])
def test_nav2_guard_ros_arguments_do_not_bypass_required_or_validated_options(command):
	with patch.object(guard.rclpy, 'init') as initialize, \
			patch.object(guard, 'RosMotorEndpoint') as endpoint, \
			pytest.raises(SystemExit) as error:
		guard.main(command)
	assert error.value.code == 2
	initialize.assert_not_called()
	endpoint.assert_not_called()
