"""모터 드라이버 + 베이스 컨트롤러를 가상 Feetech 버스(PTY)에 붙여 처음부터 끝까지 시험한다.

lekiwi/motor/power → ready → cmd_vel → 바퀴 속도 → 엔코더 → odom, 정지 · 토크 끔,
베이스 컨트롤러가 죽었을 때 드라이버가 혼자 멈추는지를 본다. 실제 모터 · 시리얼 장치를 쓰지 않는다.
설정은 실제 bringup과 같은 파일(드라이버 config, 베이스 컨트롤러 config + base_frame.yaml)을 쓴다.
"""

import importlib.util
import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from lekiwi_msgs.msg import MotorStateArray


def load_fake_bus():
	path = os.path.join(
		get_package_share_directory('lekiwi_motor_driver'), 'test_support', 'fake_sts_bus.py')
	spec = importlib.util.spec_from_file_location('fake_sts_bus', path)
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


fake = load_fake_bus()


def executable(package, name):
	return os.path.join(get_package_prefix(package), 'lib', package, name)


def yaw_of(odom):
	q = odom.pose.pose.orientation
	return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Stack:
	"""가상 버스 + 드라이버 + 베이스 컨트롤러 + 시험용 ROS 노드."""

	def __init__(self, node):
		self.node = node
		self.bus = fake.FakeStsBus()
		env = os.environ.copy()
		self.driver = subprocess.Popen(
			[
				executable('lekiwi_motor_driver', 'lekiwi_motor_driver_node'), '--ros-args',
				'--params-file', os.path.join(
					get_package_share_directory('lekiwi_motor_driver'), 'config', 'lekiwi_motor_driver.yaml'),
				'-p', f'port:={self.bus.port}', '-p', 'read_timeout_ms:=30',
				'-p', 'wheel_write_enabled:=true',
			],
			env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
		self.controller = self.start_controller(env)
		self.ready = None
		self.odom = None
		self.wheels_online_at = None
		ready_qos = QoSProfile(
			depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self.subscriptions = [
			node.create_subscription(Bool, 'lekiwi/motor/ready', self.on_ready, ready_qos),
			node.create_subscription(Odometry, 'odom', self.on_odom, 10),
			node.create_subscription(
				MotorStateArray, 'lekiwi/motor/states', self.on_states, qos_profile_sensor_data),
		]
		self.power = node.create_client(SetBool, 'lekiwi/motor/power')
		self.cmd_vel = node.create_publisher(Twist, 'cmd_vel', 10)

	@staticmethod
	def start_controller(env):
		return subprocess.Popen(
			[
				executable('lekiwi_base_controller', 'lekiwi_base_controller_node'), '--ros-args',
				'--params-file', os.path.join(
					get_package_share_directory('lekiwi_base_controller'), 'config',
					'lekiwi_base_controller.yaml'),
				'--params-file', os.path.join(
					get_package_share_directory('lekiwi_description'), 'config', 'base_frame.yaml'),
			],
			env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

	def on_ready(self, message):
		self.ready = message.data

	def on_odom(self, message):
		self.odom = message

	def on_states(self, message):
		wheels = [motor for motor in message.motors if motor.id in fake.WHEEL_IDS]
		if len(wheels) == len(fake.WHEEL_IDS) and all(motor.online for motor in wheels):
			if self.wheels_online_at is None:
				self.wheels_online_at = time.monotonic()

	def wait_until_armable(self, timeout=20.0):
		"""드라이버 상태가 흐르기 시작하고 컨트롤러도 받을 시간(1 s)이 지날 때까지 기다린다.

		그 전에 power를 부르면 컨트롤러가 '연결 안 됨'으로 고장을 고정한다 (lekiwi_node와 같은 규칙).
		"""
		return self.spin_until(
			lambda: self.ready is False and self.odom is not None and self.wheels_online_at is not None
			and time.monotonic() - self.wheels_online_at > 1.0,
			timeout)

	def spin_until(self, predicate, timeout):
		end = time.monotonic() + timeout
		while time.monotonic() < end:
			if predicate():
				return True
			rclpy.spin_once(self.node, timeout_sec=0.05)
		return predicate()

	def set_power(self, is_enabled):
		assert self.power.wait_for_service(timeout_sec=15.0), 'lekiwi/motor/power missing'
		future = self.power.call_async(SetBool.Request(data=is_enabled))
		assert self.spin_until(future.done, 10.0), 'power service did not answer'
		return future.result()

	def drive(self, x, y, wz, duration):
		twist = Twist()
		twist.linear.x, twist.linear.y, twist.angular.z = x, y, wz
		end = time.monotonic() + duration
		while time.monotonic() < end:
			self.cmd_vel.publish(twist)
			rclpy.spin_once(self.node, timeout_sec=0.05)

	def wheel_speeds(self):
		return [self.bus.goal_speed(motor_id) for motor_id in fake.WHEEL_IDS]

	def wheel_torques(self):
		return [self.bus.register(motor_id, fake.ADDR_TORQUE) for motor_id in fake.WHEEL_IDS]

	def pose(self):
		position = self.odom.pose.pose.position
		return position.x, position.y, yaw_of(self.odom)

	def stop(self):
		output = ''
		for process in (self.controller, self.driver):
			if process.poll() is None:
				process.send_signal(signal.SIGINT)
				try:
					process.wait(timeout=10.0)
				except subprocess.TimeoutExpired:
					process.kill()
			output += process.stdout.read()
		for subscription in self.subscriptions:
			self.node.destroy_subscription(subscription)
		self.node.destroy_client(self.power)
		self.node.destroy_publisher(self.cmd_vel)
		self.bus.close()
		return output


@pytest.fixture(scope='module')
def ros_node():
	# LEKIWI_TEST_ROS_DOMAIN_ID가 있으면 그 도메인을 쓴다 (함께 도는 다른 시험 · 로봇과 나눈다)
	os.environ['ROS_DOMAIN_ID'] = os.environ.get('LEKIWI_TEST_ROS_DOMAIN_ID', str(60 + os.getpid() % 20))
	os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
	rclpy.init()
	node = Node('test_motor_stack_client')
	yield node
	node.destroy_node()
	rclpy.shutdown()


@pytest.fixture
def stack(ros_node):
	under_test = Stack(ros_node)
	yield under_test
	print(under_test.stop())


def test_power_drive_odometry_and_stop(stack):
	# 시작: 토크 꺼짐, ready=false
	assert stack.wait_until_armable()
	assert stack.wheel_torques() == [0, 0, 0]

	result = stack.set_power(True)
	assert result.success, result.message
	assert stack.spin_until(lambda: stack.ready is True, 5.0)
	assert stack.wheel_torques() == [1, 1, 1]

	# 전진 0.05 m/s × 2 s → odom x ≈ 0.10 m (명령 → 바퀴 → 엔코더 → odom 왕복)
	x0, y0, yaw0 = stack.pose()
	stack.drive(0.05, 0.0, 0.0, 2.0)
	assert any(speed != 0 for speed in stack.wheel_speeds())
	assert stack.spin_until(lambda: stack.wheel_speeds() == [0, 0, 0], 1.0), 'cmd_vel timeout did not stop'
	stack.spin_until(lambda: False, 0.5)
	x1, y1, yaw1 = stack.pose()
	assert 0.07 < x1 - x0 < 0.13, (x1 - x0)
	assert abs(y1 - y0) < 0.02 and abs(yaw1 - yaw0) < 0.05

	# 옆으로 0.05 m/s × 1 s → odom y ≈ 0.05 m
	stack.drive(0.0, 0.05, 0.0, 1.0)
	stack.spin_until(lambda: False, 0.8)
	x2, y2, yaw2 = stack.pose()
	assert 0.03 < y2 - y1 < 0.07, (y2 - y1)
	assert abs(x2 - x1) < 0.02

	# 제자리 회전 0.3 rad/s × 1 s → yaw ≈ 0.3 rad
	stack.drive(0.0, 0.0, 0.3, 1.0)
	stack.spin_until(lambda: False, 0.8)
	_, _, yaw3 = stack.pose()
	assert 0.2 < yaw3 - yaw2 < 0.4, (yaw3 - yaw2)

	# 전원 끔 → 토크 꺼짐, ready=false
	result = stack.set_power(False)
	assert result.success, result.message
	assert stack.spin_until(lambda: stack.ready is False, 2.0)
	assert stack.wheel_torques() == [0, 0, 0]
	# 팔에는 어떤 쓰기도 가지 않았다
	assert stack.bus.written_ids().isdisjoint(fake.ARM_IDS)


def test_drive_at_teleop_speed_uses_the_raised_wheel_limit(stack):
	# 0.2 m/s 직진이면 바퀴 하나가 약 3.5 rad/s (tick 약 2260)로 드라이버 코드 기본 상한 1500을 넘는다.
	# 드라이버 설정(speed_tick_limit 3000)과 베이스 컨트롤러 설정(max_wheel_speed)이 짝이어야 그대로 나온다.
	assert stack.wait_until_armable()
	assert stack.set_power(True).success
	assert stack.spin_until(lambda: stack.ready is True, 5.0)
	x0, y0, yaw0 = stack.pose()
	stack.drive(0.2, 0.0, 0.0, 1.0)
	assert max(abs(speed) for speed in stack.wheel_speeds()) > 1500, stack.wheel_speeds()
	stack.spin_until(lambda: False, 0.8)
	x1, y1, yaw1 = stack.pose()
	assert 0.15 < x1 - x0 < 0.3, (x1 - x0)
	assert abs(y1 - y0) < 0.03 and abs(yaw1 - yaw0) < 0.05


def test_driver_stops_wheels_when_controller_dies(stack):
	assert stack.wait_until_armable()
	assert stack.set_power(True).success
	assert stack.spin_until(lambda: stack.ready is True, 5.0)
	stack.drive(0.05, 0.0, 0.0, 0.5)
	assert any(speed != 0 for speed in stack.wheel_speeds())

	# 베이스 컨트롤러가 정리할 틈 없이 죽는다
	stack.controller.kill()
	killed_at = time.monotonic()
	assert stack.spin_until(lambda: stack.wheel_speeds() == [0, 0, 0], 1.5)
	assert time.monotonic() - killed_at < 1.5
	assert stack.spin_until(lambda: stack.wheel_torques() == [0, 0, 0], 5.0)
	assert time.monotonic() - killed_at >= 2.5   # 토크 끔은 3 s 감시가 처리한다


def test_driver_restart_latches_fault_until_power_cycle(stack):
	assert stack.wait_until_armable()
	assert stack.set_power(True).success
	assert stack.spin_until(lambda: stack.ready is True, 5.0)

	# 드라이버가 사라지면 엔코더가 끊겨 고장이 고정된다
	stack.driver.send_signal(signal.SIGINT)
	stack.driver.wait(timeout=10.0)
	assert stack.spin_until(lambda: stack.ready is False, 3.0)
	# 드라이버가 종료하면서 토크를 껐다
	assert stack.wheel_torques() == [0, 0, 0]
	# 드라이버가 없으면 다시 켜기를 거부한다
	assert not stack.set_power(True).success
