"""lekiwi_zmq_host_node 통합 시험: 가짜 로봇 노드들 + 바닐라 lerobot LeKiwiClient (하드웨어 없음).

rclpy · zmq · lerobot · lekiwi_msgs 가 모두 있어야 돈다 (없으면 건너뛴다):
	source install/setup.bash && ~/miniconda3/envs/physical-labs/bin/python -m pytest src/lekiwi_zmq_host/test
다른 ROS 그래프와 섞이지 않게 도메인 180~189 · LOCALHOST 로 띄운다.
"""

import array
import importlib.util
import json
import os
import socket
import threading
import time

import pytest

# 모듈 단위 importorskip 은 쓰지 않는다: ROS 의 launch_testing pytest 플러그인이 그 skip 을 받아
# 같은 디렉터리의 다른 시험까지 모으지 않는다 (시스템 python 3 · pytest 7.4)
MISSING = [
	name for name in ('rclpy', 'zmq', 'lekiwi_msgs', 'lerobot', 'cv2', 'numpy')
	if importlib.util.find_spec(name) is None]
pytestmark = pytest.mark.skipif(bool(MISSING), reason=f'없는 모듈: {", ".join(MISSING)}')

if not MISSING:
	os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 10)
	os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

	import cv2
	import numpy as np
	import rclpy
	from geometry_msgs.msg import Twist
	from lekiwi_msgs.msg import LerobotState
	from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
	from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient
	from rclpy.executors import SingleThreadedExecutor
	from rclpy.node import Node
	from rclpy.parameter import Parameter
	from rclpy.qos import qos_profile_sensor_data
	from sensor_msgs.msg import CompressedImage, JointState
	from std_msgs.msg import Bool, String
	from std_srvs.srv import SetBool

	from lekiwi_zmq_host.node import LATCHED_QOS, ZmqHostNode
	from lekiwi_zmq_host.protocol import ARM_JOINTS, ARM_KEYS, STATE_NAMES

	ARM_ACTION = {key: float(index) * 5.0 for index, key in enumerate(ARM_KEYS)}

STATE_VALUES = [10.0, -20.0, 30.0, 40.0, -50.0, 60.0, 0.0, 0.0, 0.0]


def free_port() -> int:
	with socket.socket() as sock:
		sock.bind(('127.0.0.1', 0))
		return sock.getsockname()[1]


def wait_for(condition, timeout=5.0, step=None):
	deadline = time.monotonic() + timeout
	while time.monotonic() < deadline:
		if step is not None:
			step()
		if condition():
			return True
		time.sleep(0.03)
	return False


class FakeRobot:
	"""bringup 의 노드 이름 · 토픽 · 서비스를 흉내 낸다 (드라이버 · 베이스 컨트롤러 · 팔로워 · 카메라 · 게이트웨이)."""

	def __init__(self) -> None:
		self.power_requests: list[bool] = []
		self.arm_requests: list[bool] = []
		self.cmd_vel: list[tuple[float, float, float]] = []
		self.goals: list[JointState] = []
		self.statuses: list[dict] = []
		self.is_publishing_state = True

		self.base = Node('lekiwi_base_controller_node', parameter_overrides=[
			Parameter('wheel_radius', value=0.05), Parameter('base_radius', value=0.13647),
			Parameter('kinematics_frame_x', value=0.007587843843),
			Parameter('kinematics_frame_y', value=-0.010776931094),
			Parameter('kinematics_frame_yaw', value=2.094379865161),
			Parameter('wheel_ids', value=[8, 9, 7]), Parameter('wheel_directions', value=[1.0, 1.0, 1.0]),
		], automatically_declare_parameters_from_overrides=True)
		self._ready_pub = self.base.create_publisher(Bool, 'lekiwi/motor/ready', LATCHED_QOS)
		self._ready_pub.publish(Bool(data=False))
		self.base.create_service(SetBool, 'lekiwi/motor/power', self._power)
		self.base.create_subscription(
			Twist, 'cmd_vel', lambda m: self.cmd_vel.append((m.linear.x, m.linear.y, m.angular.z)), 10)

		self.driver = Node('lekiwi_motor_driver_node')
		self._arm_ready_pub = self.driver.create_publisher(Bool, 'lekiwi/motor/arm_ready', LATCHED_QOS)
		self._arm_ready_pub.publish(Bool(data=False))
		self.driver.create_service(SetBool, 'lekiwi/motor/arm_torque', self._arm_torque)

		self.follower = Node('lekiwi_follower_arm_node')
		self._state_pub = self.follower.create_publisher(LerobotState, 'lekiwi/lerobot/state', 10)
		self._arm_status_pub = self.follower.create_publisher(String, 'lekiwi/arm/status', 10)
		self.follower.create_subscription(
			JointState, 'lekiwi/arm/goal', self.goals.append, qos_profile_sensor_data)
		self.follower.create_timer(1.0 / 30.0, self._follower_tick)

		self.camera = Node('front_camera_node')
		jpeg = cv2.imencode('.jpg', np.full((480, 640, 3), 128, np.uint8))[1].tobytes()
		self._image = CompressedImage(format='jpeg', data=array.array('B', jpeg))
		self._camera_pubs = [
			self.camera.create_publisher(
				CompressedImage, f'lekiwi/sensor/{name}_camera/image_raw/compressed', 2)
			for name in ('front', 'wrist')]
		self.camera.create_timer(1.0 / 30.0, self._camera_tick)

		self.gateway = Node('lekiwi_web_operator')
		self.gateway.create_publisher(Bool, 'lekiwi/safety/estop_latched', LATCHED_QOS).publish(Bool(data=False))
		self.gateway.create_subscription(
			String, 'lekiwi/zmq_host/status', lambda m: self.statuses.append(json.loads(m.data)), LATCHED_QOS)

		self.nodes = [self.base, self.driver, self.follower, self.camera, self.gateway]

	def _power(self, request, response):
		self.power_requests.append(request.data)
		self._ready_pub.publish(Bool(data=False))   # 실물처럼: 켜면 엔코더를 기다리는 동안 false
		if request.data:
			self._ready_pub.publish(Bool(data=True))
		response.success = True
		return response

	def _arm_torque(self, request, response):
		self.arm_requests.append(request.data)
		self._arm_ready_pub.publish(Bool(data=request.data))
		response.success = True
		return response

	def _follower_tick(self):
		self._arm_status_pub.publish(String(data=json.dumps({'state': 'following', 'reason': ''})))
		if self.is_publishing_state:
			self._state_pub.publish(LerobotState(names=list(STATE_NAMES), values=STATE_VALUES))

	def _camera_tick(self):
		for pub in self._camera_pubs:
			pub.publish(self._image)


@pytest.fixture
def system():
	rclpy.init()
	cmd_port, obs_port = free_port(), free_port()
	robot = FakeRobot()
	host = ZmqHostNode(parameter_overrides=[
		Parameter('port_zmq_cmd', value=cmd_port), Parameter('port_zmq_observations', value=obs_port)])
	executor = SingleThreadedExecutor()
	for node in [host, *robot.nodes]:
		executor.add_node(node)
	thread = threading.Thread(target=executor.spin, daemon=True)
	thread.start()
	client = LeKiwiClient(LeKiwiClientConfig(
		remote_ip='127.0.0.1', port_zmq_cmd=cmd_port, port_zmq_observations=obs_port, id='zmq_host_test'))
	try:
		assert wait_for(lambda: host._base is not None), '베이스 컨트롤러 파라미터를 읽지 못했다'
		client.connect()
		yield host, robot, client
	finally:
		if client.zmq_context is not None:
			# connect 가 실패해도 닫는다 (안 닫은 zmq 컨텍스트는 GC 에서 term 이 영원히 기다린다)
			client.zmq_context.destroy(linger=0)
		executor.shutdown()
		thread.join(timeout=2.0)
		for node in [host, *robot.nodes]:
			node.destroy_node()
		rclpy.try_shutdown()


def drive(client, seconds, action):
	deadline = time.monotonic() + seconds
	while time.monotonic() < deadline:
		client.send_action(action)
		client.get_observation()
		time.sleep(1.0 / 30.0)


def last_status(robot):
	return robot.statuses[-1] if robot.statuses else {}


def test_vanilla_lekiwi_client_observes_drives_and_stops(system):
	host, robot, client = system

	# 1) 관측: 상태 9개 + 카메라 2대
	observation = {}

	def observe():
		observation.clear()
		observation.update(client.get_observation())

	assert wait_for(lambda: 'front' in observation and 'wrist' in observation, step=observe)
	assert [observation[name] for name in STATE_NAMES] == pytest.approx(STATE_VALUES)
	assert observation['front'].shape == (480, 640, 3)
	assert last_status(robot)['session'] == 'idle'

	# 2) 명령: 팔 목표 · cmd_vel · 토크
	drive(client, 1.0, {**ARM_ACTION, 'x.vel': 0.1, 'y.vel': 0.0, 'theta.vel': 0.0})
	assert robot.power_requests == [True] and robot.arm_requests == [True]
	goal = robot.goals[-1]
	assert list(goal.name) == list(ARM_JOINTS)
	assert list(goal.position) == pytest.approx(list(ARM_ACTION.values()))
	assert robot.cmd_vel[-1] == pytest.approx((0.1, 0.0, 0.0), abs=1e-4)
	assert last_status(robot) == {
		'session': 'active', 'reason': None, 'clients': 1, 'client_ip': '127.0.0.1', 'estop_unknown': False}

	# 3) 명령이 끊기면 0 → 2 s 에 세션 끝 (바퀴 토크 끔, 팔은 그대로)
	assert wait_for(lambda: last_status(robot).get('session') == 'idle', timeout=3.0, step=client.get_observation)
	assert robot.cmd_vel[-1] == (0.0, 0.0, 0.0)
	assert robot.power_requests == [True, False] and robot.arm_requests == [True]
	assert host._cmd_vel_pub is None

	# 4) 웹 화면이 조작 중이면 시작을 거절한다
	view = Node('lekiwi_web_operator_view')
	try:
		assert wait_for(lambda: 'lekiwi_web_operator_view' in host.get_node_names())
		drive(client, 0.5, {'x.vel': 0.1})
		assert last_status(robot)['session'] == 'blocked'
		assert last_status(robot)['reason'] == '웹 화면이 조작 중입니다'
		assert robot.power_requests == [True, False]
	finally:
		view.destroy_node()

	# 5) 상태가 끊기면 관측을 보내지 않는다 (LeKiwiClient 가 0 자세로 채우지 않게)
	robot.is_publishing_state = False
	time.sleep(0.7)
	client._poll_and_get_latest_message()   # 끊기기 전에 보낸 마지막 하나
	deadline = time.monotonic() + 0.6
	while time.monotonic() < deadline:
		assert client._poll_and_get_latest_message() is None
	assert wait_for(lambda: '관측 상태' in (last_status(robot).get('reason') or ''), timeout=3.0)
