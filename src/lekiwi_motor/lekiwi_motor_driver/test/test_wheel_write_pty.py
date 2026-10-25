"""가상 Feetech 버스(PTY)에 lekiwi_motor_driver_node를 붙여 바퀴 · 팔 쓰기와 안전 처리를 시험한다.

실제 모터 · 시리얼 장치를 쓰지 않는다. 가상 버스는 STS3215 9개(팔 1~6, 바퀴 7~9)의
레지스터를 흉내 내고, 노드가 보낸 WRITE · SYNC_WRITE를 모두 기록한다.
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from lekiwi_msgs.msg import ArmCommand, ArmRegisters, MotorStateArray, WheelCommand

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_sts_bus import (  # noqa: E402
	ADDR_D_COEFFICIENT, ADDR_GOAL_POSITION, ADDR_GOAL_SPEED, ADDR_HOMING_OFFSET, ADDR_I_COEFFICIENT,
	ADDR_MAX_POSITION_LIMIT, ADDR_MIN_POSITION_LIMIT, ADDR_MODE, ADDR_P_COEFFICIENT, ADDR_POSITION,
	ADDR_STATUS, ADDR_TORQUE, ARM_IDS, STATUS_OVERHEAT, STATUS_OVERLOAD, TICKS_PER_RAD, WHEEL_IDS,
	FakeStsBus)

ARM_PRESENT = {motor_id: 1000 + 100 * motor_id for motor_id in ARM_IDS}   # 1100 ~ 1600

@pytest.fixture(scope='module')
def ros_node():
	# LEKIWI_TEST_ROS_DOMAIN_ID가 있으면 그 도메인을 쓴다 (함께 도는 다른 시험 · 로봇과 나눈다)
	os.environ['ROS_DOMAIN_ID'] = os.environ.get('LEKIWI_TEST_ROS_DOMAIN_ID', str(80 + os.getpid() % 20))
	os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
	rclpy.init()
	node = Node('test_wheel_write_client')
	yield node
	node.destroy_node()
	rclpy.shutdown()


class DriverUnderTest:
	def __init__(self, ros_node, bus, is_write_enabled=True, is_arm_write_enabled=False, params=()):
		self.node = ros_node
		self.bus = bus
		executable = os.environ['LEKIWI_MOTOR_DRIVER_EXE']
		self.process = subprocess.Popen(
			[
				executable, '--ros-args',
				'-p', f'port:={bus.port}',
				'-p', 'read_timeout_ms:=30',
				'-p', f'wheel_write_enabled:={"true" if is_write_enabled else "false"}',
				'-p', f'arm_write_enabled:={"true" if is_arm_write_enabled else "false"}',
			] + [argument for param in params for argument in ('-p', param)],
			env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
		self.client = ros_node.create_client(SetBool, 'lekiwi/motor/wheel_torque')
		self.publisher = ros_node.create_publisher(WheelCommand, 'lekiwi/motor/wheel_command', 1)
		self.arm_client = ros_node.create_client(SetBool, 'lekiwi/motor/arm_torque')
		self.arm_publisher = ros_node.create_publisher(ArmCommand, 'lekiwi/motor/arm_command', 1)
		self.states = []
		self.arm_ready = []       # lekiwi/motor/arm_ready 값을 받은 순서대로
		self.arm_registers = []   # lekiwi/motor/arm_registers 메시지
		latched = QoSProfile(
			depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self.subscriptions = [
			ros_node.create_subscription(
				MotorStateArray, 'lekiwi/motor/states', self.states.append, qos_profile_sensor_data),
			ros_node.create_subscription(
				Bool, 'lekiwi/motor/arm_ready', lambda message: self.arm_ready.append(message.data), latched),
			ros_node.create_subscription(
				ArmRegisters, 'lekiwi/motor/arm_registers', self.arm_registers.append, latched),
		]

	def wait_for_states(self, timeout=10.0):
		is_ready = wait_until(self.node, lambda: len(self.states) > 0, timeout)
		# 시작할 때 노드가 보낸 정리 쓰기(속도 0, 토크 끔) 다음부터 센다
		self.startup_writes = len(self.bus.writes)
		return is_ready

	def writes_after_startup(self):
		return self.bus.writes[self.startup_writes:]

	def call_torque(self, is_enabled, timeout=10.0, client=None):
		client = client or self.client
		assert client.wait_for_service(timeout_sec=timeout), f'{client.srv_name} service missing'
		future = client.call_async(SetBool.Request(data=is_enabled))
		rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
		assert future.done(), f'{client.srv_name} service did not answer'
		return future.result()

	def call_arm_torque(self, is_enabled):
		return self.call_torque(is_enabled, client=self.arm_client)

	def command(self, ids, velocity):
		self.publisher.publish(WheelCommand(ids=ids, velocity=velocity))

	def arm_command(self, ids, positions):
		self.arm_publisher.publish(ArmCommand(ids=ids, positions_ticks=positions))

	def stream(self, ids, velocity, duration, rate=20.0, send=None):
		send = send or self.command
		end = time.monotonic() + duration
		while time.monotonic() < end:
			send(ids, velocity)
			rclpy.spin_once(self.node, timeout_sec=1.0 / rate)

	def arm_stream(self, ids, positions, duration):
		self.stream(ids, positions, duration, send=self.arm_command)

	def stop(self):
		if self.process.poll() is None:
			self.process.send_signal(signal.SIGINT)
			try:
				self.process.wait(timeout=10.0)
			except subprocess.TimeoutExpired:
				self.process.kill()
		self.node.destroy_client(self.client)
		self.node.destroy_publisher(self.publisher)
		self.node.destroy_client(self.arm_client)
		self.node.destroy_publisher(self.arm_publisher)
		for subscription in self.subscriptions:
			self.node.destroy_subscription(subscription)
		return self.process.stdout.read()


def wait_until(node, predicate, timeout):
	end = time.monotonic() + timeout
	while time.monotonic() < end:
		if predicate():
			return True
		rclpy.spin_once(node, timeout_sec=0.05)
	return predicate()


def spin_for(node, duration):
	wait_until(node, lambda: False, duration)


def place_arm(bus, stale_goal=3000):
	"""팔 모터가 서로 다른 자리에 있고, 목표 레지스터에는 옛 값이 남아 있다."""
	for motor_id, ticks in ARM_PRESENT.items():
		bus.set_word(motor_id, ADDR_POSITION, ticks)
		bus.set_word(motor_id, ADDR_GOAL_POSITION, stale_goal)


def goal_position_writes(writes):
	"""팔 Goal_Position SYNC_WRITE마다 (ID, tick) 목록"""
	return [
		[(motor_id, int.from_bytes(data, 'little')) for motor_id, data in write[3]]
		for write in writes if write[0] == 0x83 and write[2] == ADDR_GOAL_POSITION]


@pytest.fixture
def bus():
	fake = FakeStsBus()
	yield fake
	fake.close()


def test_read_only_by_default(ros_node, bus):
	bus.set_register(7, ADDR_TORQUE, 1)   # 다른 프로그램이 켜 둔 토크도 읽기 전용이면 건드리지 않는다
	driver = DriverUnderTest(ros_node, bus, is_write_enabled=False)
	try:
		assert driver.wait_for_states()
		assert not driver.client.wait_for_service(timeout_sec=2.0)
		assert not driver.arm_client.wait_for_service(timeout_sec=0.5)
		driver.command(WHEEL_IDS, [1.0, 1.0, 1.0])
		driver.arm_command(ARM_IDS, [2048] * len(ARM_IDS))
		spin_for(ros_node, 0.5)
		assert bus.writes == []
		assert bus.register(7, ADDR_TORQUE) == 1
		assert all(motor.online for motor in driver.states[-1].motors)
	finally:
		driver.stop()


def test_startup_stops_wheels_left_running(ros_node, bus):
	# 이전 프로그램이 바퀴를 돌리던 채로 남겼다. 팔 토크도 켜져 있다.
	for motor_id in WHEEL_IDS:
		bus.set_register(motor_id, ADDR_TORQUE, 1)
		bus.set_register(motor_id, ADDR_GOAL_SPEED, 100)
	bus.set_register(1, ADDR_TORQUE, 1)
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
		assert all(bus.goal_speed(motor_id) == 0 for motor_id in WHEEL_IDS)
		assert bus.register(1, ADDR_TORQUE) == 1
		assert bus.written_ids().isdisjoint(ARM_IDS)
	finally:
		output = driver.stop()
	assert 'startup: wheel speed zeroed and torque disabled' in output


def test_command_ignored_while_torque_off(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert driver.client.wait_for_service(timeout_sec=10.0)
		driver.stream(WHEEL_IDS, [1.0, 1.0, 1.0], 0.5)
		assert driver.writes_after_startup() == []
	finally:
		driver.stop()


def test_enable_refused_when_wheel_not_in_velocity_mode(ros_node, bus):
	bus.set_register(8, ADDR_MODE, 0)
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		result = driver.call_torque(True)
		assert not result.success
		assert 'velocity mode' in result.message
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
		assert driver.writes_after_startup() == []
	finally:
		driver.stop()


def test_enable_refused_on_status_fault(ros_node, bus):
	bus.set_register(7, ADDR_STATUS, STATUS_OVERHEAT)
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		result = driver.call_torque(True)
		assert not result.success
		assert 'overheat' in result.message
		assert driver.writes_after_startup() == []
	finally:
		driver.stop()


def test_enable_command_and_watchdog(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		result = driver.call_torque(True)
		assert result.success, result.message
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in WHEEL_IDS)
		# 토크를 켜기 전에 속도 0을 먼저 썼다
		first = driver.writes_after_startup()[0]
		assert first[0] == 0x83 and first[2] == ADDR_GOAL_SPEED
		assert all(data == b'\x00\x00' for _, data in first[3])

		driver.stream([8, 9, 7], [1.0, -1.0, 0.5], 1.0)
		assert bus.goal_speed(8) == round(1.0 * TICKS_PER_RAD)
		assert bus.goal_speed(9) == -round(1.0 * TICKS_PER_RAD)
		assert bus.goal_speed(7) == round(0.5 * TICKS_PER_RAD)

		# 명령이 끊기면 0.5 s 뒤 속도 0, 3 s 뒤 토크 끔
		stopped_at = time.monotonic()
		assert wait_until(ros_node, lambda: all(bus.goal_speed(i) == 0 for i in WHEEL_IDS), 2.0)
		assert time.monotonic() - stopped_at >= 0.4
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in WHEEL_IDS)
		assert wait_until(
			ros_node, lambda: all(bus.register(i, ADDR_TORQUE) == 0 for i in WHEEL_IDS), 5.0)
		assert time.monotonic() - stopped_at >= 2.8

		# 팔에는 어떤 쓰기도 가지 않았다
		assert bus.written_ids().isdisjoint(ARM_IDS)
	finally:
		driver.stop()


def test_rejects_commands_with_arm_or_repeated_ids(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert driver.call_torque(True).success
		driver.stream([1, 8], [1.0, 1.0], 0.5)
		driver.stream([8, 8], [1.0, 1.0], 0.5)
		driver.stream([8], [float('nan')], 0.5)
		assert bus.goal_speed(8) == 0
		assert bus.written_ids().isdisjoint(ARM_IDS)
	finally:
		driver.stop()


def test_wheel_dropout_stops_all_wheels(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert driver.call_torque(True).success
		driver.stream(WHEEL_IDS, [1.0, 1.0, 1.0], 0.5)
		bus.set_responsive(9, False)
		driver.stream(WHEEL_IDS, [1.0, 1.0, 1.0], 1.0)
		assert bus.register(7, ADDR_TORQUE) == 0 and bus.register(8, ADDR_TORQUE) == 0
		assert bus.goal_speed(7) == 0 and bus.goal_speed(8) == 0
	finally:
		output = driver.stop()
	assert 'stopped responding' in output


def test_external_torque_off_stops_all_wheels(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert driver.call_torque(True).success
		bus.set_register(8, ADDR_TORQUE, 0)
		driver.stream(WHEEL_IDS, [1.0, 1.0, 1.0], 2.0)
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
		assert all(bus.goal_speed(motor_id) == 0 for motor_id in WHEEL_IDS)
	finally:
		output = driver.stop()
	assert 'torque turned off outside this driver' in output


def test_disable_service_and_shutdown(ros_node, bus):
	driver = DriverUnderTest(ros_node, bus)
	try:
		assert driver.wait_for_states()
		assert driver.call_torque(True).success
		result = driver.call_torque(False)
		assert result.success, result.message
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
		assert driver.call_torque(True).success
		driver.stream(WHEEL_IDS, [1.0, 1.0, 1.0], 0.3)
	finally:
		output = driver.stop()
	# 토크가 켜진 채 종료(SIGINT)하면 노드가 속도 0 → 토크 끔을 보낸다
	assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
	assert all(bus.goal_speed(motor_id) == 0 for motor_id in WHEEL_IDS)
	assert 'shutdown: wheel speed zeroed and torque disabled' in output


# ---------------------------------------------------------------------------
# 팔 쓰기
# ---------------------------------------------------------------------------

def test_arm_startup_keeps_torque_and_ignores_commands_until_enabled(ros_node, bus):
	# 다른 프로그램이 팔 토크를 켜 둔 채 남겼다. 드라이버는 시작 · 종료할 때 팔을 건드리지 않는다.
	place_arm(bus)
	for motor_id in ARM_IDS:
		bus.set_register(motor_id, ADDR_TORQUE, 1)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.arm_client.wait_for_service(timeout_sec=10.0)
		# 서비스로 켜지 않았으므로 명령을 버린다
		driver.arm_stream(ARM_IDS, [2048] * len(ARM_IDS), 0.5)
		assert all(bus.goal_position(motor_id) == 3000 for motor_id in ARM_IDS)
	finally:
		output = driver.stop()
	assert bus.written_ids().isdisjoint(ARM_IDS)
	assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in ARM_IDS)
	assert 'arm write ready for ids [1, 2, 3, 4, 5, 6]' in output
	assert 'arm command ignored: arm is not ready' in output


def test_arm_torque_on_sets_goal_to_present_before_torque(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in ARM_IDS)
		assert all(bus.goal_position(motor_id) == ARM_PRESENT[motor_id] for motor_id in ARM_IDS)
		# 팔 ID에만 하나씩 WRITE (SYNC_WRITE · 브로드캐스트 없음). 목표를 모두 쓴 뒤 토크를 켰다
		writes = driver.writes_after_startup()
		assert [(write[0], write[1], write[2]) for write in writes] == (
			[(0x03, motor_id, ADDR_GOAL_POSITION) for motor_id in ARM_IDS] +
			[(0x03, motor_id, ADDR_TORQUE) for motor_id in ARM_IDS])
		assert all(write[3] == b'\x01' for write in writes[len(ARM_IDS):])
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in WHEEL_IDS)
	finally:
		driver.stop()


def test_arm_torque_refused_when_arm_not_in_position_mode(ros_node, bus):
	bus.set_register(3, ADDR_MODE, 1)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		result = driver.call_arm_torque(True)
		assert not result.success
		assert 'position mode' in result.message
		assert driver.writes_after_startup() == []
	finally:
		driver.stop()


def test_arm_command_writes_every_goal_and_clamps(ros_node, bus):
	place_arm(bus)   # 지금 위치 1100 ~ 1600. 가상 팔은 움직이지 않는다
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		# SYNC_WRITE는 ACK가 없으므로 같은 명령이라도 받을 때마다 모든 ID를 한 패킷으로 다시 쓴다.
		# 2번: 4095로 자른 뒤 지금 위치 1200 + 1024, 3번: 0으로 자른 뒤 1300 - 1024, 5번: 1500 + 1024
		mark = len(bus.writes)
		driver.arm_stream(ARM_IDS, [1150, 5000, -20, 1400, 3000, 1450], 0.5)
		spin_for(ros_node, 0.2)
		writes = goal_position_writes(bus.writes[mark:])
		assert len(writes) >= 5
		assert len(bus.writes) - mark == len(writes)
		assert all(
			write == [(1, 1150), (2, 2224), (3, 276), (4, 1400), (5, 2524), (6, 1450)] for write in writes)
		# 일부 ID만 보내도 된다. 받은 ID만 쓴다
		mark = len(bus.writes)
		driver.arm_stream([6, 1], [1500, 1000], 0.3)
		spin_for(ros_node, 0.2)
		writes = goal_position_writes(bus.writes[mark:])
		assert writes and all(write == [(6, 1500), (1, 1000)] for write in writes)
	finally:
		output = driver.stop()
	assert 'positions outside [0, 4095] were clamped' in output
	assert 'goals more than 1024 ticks from the present position were clamped' in output


def test_arm_rejects_commands_with_wheel_or_repeated_ids(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		mark = len(bus.writes)
		driver.arm_stream([1, 7], [1500, 1500], 0.5)
		driver.arm_stream([1, 1], [1500, 1600], 0.5)
		driver.arm_stream([1, 2], [1500], 0.5)
		assert bus.writes[mark:] == []
		assert all(bus.goal_position(motor_id) == ARM_PRESENT[motor_id] for motor_id in ARM_IDS)
	finally:
		driver.stop()


def test_arm_torque_off_touches_only_arm_ids_and_stops_commands(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		# 바퀴 토크가 켜져 있어도 팔 끄기는 바퀴를 건드리지 않는다
		for motor_id in WHEEL_IDS:
			bus.set_register(motor_id, ADDR_TORQUE, 1)
		mark = len(bus.writes)
		result = driver.call_arm_torque(False)
		assert result.success, result.message
		assert bus.writes[mark:] == [(0x03, motor_id, ADDR_TORQUE, b'\x00') for motor_id in ARM_IDS]
		assert all(bus.register(motor_id, ADDR_TORQUE) == 0 for motor_id in ARM_IDS)
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in WHEEL_IDS)
		# 끈 뒤의 명령은 버린다
		driver.arm_stream(ARM_IDS, [2048] * len(ARM_IDS), 0.5)
		assert len(bus.writes) == mark + len(ARM_IDS)
	finally:
		driver.stop()


def test_arm_and_wheel_commands_share_the_bus(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_torque(True).success
		assert driver.call_arm_torque(True).success
		# 바퀴 20 Hz와 팔 명령을 섞어 보낸다
		ticks = ARM_PRESENT[1]
		end = time.monotonic() + 1.0
		while time.monotonic() < end:
			ticks += 5   # 1 s에 약 100 tick: 가상 팔은 움직이지 않으므로 1024 tick 제한 안에 둔다
			driver.command([8, 9, 7], [1.0, -1.0, 0.5])
			driver.arm_command([1], [ticks])
			rclpy.spin_once(ros_node, timeout_sec=0.05)
		assert bus.goal_speed(8) == round(1.0 * TICKS_PER_RAD)
		assert bus.goal_speed(9) == -round(1.0 * TICKS_PER_RAD)
		assert bus.goal_speed(7) == round(0.5 * TICKS_PER_RAD)
		assert wait_until(ros_node, lambda: bus.goal_position(1) == ticks, 2.0)
		# 바퀴 쓰기는 바퀴에만, 팔 쓰기는 팔에만 갔다
		for instruction, motor_id, address, data in driver.writes_after_startup():
			ids = {entry_id for entry_id, _ in data} if instruction == 0x83 else {motor_id}
			allowed = {ADDR_GOAL_SPEED: WHEEL_IDS, ADDR_GOAL_POSITION: ARM_IDS}.get(address, ARM_IDS + WHEEL_IDS)
			assert ids <= set(allowed), (instruction, address, ids)
	finally:
		driver.stop()


def torque_writes(writes):
	"""Torque_Enable WRITE마다 (ID, 값)"""
	return [(write[1], write[3][0]) for write in writes if write[0] == 0x03 and write[2] == ADDR_TORQUE]


def test_arm_enable_again_while_on_writes_nothing(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		mark = len(bus.writes)
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert 'already enabled' in result.message
		assert bus.writes[mark:] == []
		# 명령은 계속 받는다
		driver.arm_stream([1], [1150], 0.3)
		assert wait_until(ros_node, lambda: bus.goal_position(1) == 1150, 1.0)

		# 팔 하나의 토크가 밖에서 꺼졌으면 그 팔만 다시 켠다 (목표 = 지금 위치 → 토크). 잡고 있던 팔은 건드리지 않는다
		bus.set_register(3, ADDR_TORQUE, 0)
		mark = len(bus.writes)
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert [(write[1], write[2]) for write in bus.writes[mark:]] == [
			(3, ADDR_GOAL_POSITION), (3, ADDR_TORQUE)]
		assert bus.register(3, ADDR_TORQUE) == 1
		assert bus.goal_position(1) == 1150

		# 물건을 쥔 그리퍼의 과부하: 그대로 "이미 켜짐" (다시 시작해도 원격 조작이 끊기지 않는다)
		bus.set_register(6, ADDR_STATUS, STATUS_OVERLOAD)
		mark = len(bus.writes)
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert 'already enabled' in result.message
		# 과열 등은 알린다. 아무것도 쓰지 않고 arm_ready도 그대로다 (팔은 계속 잡고, 명령도 받는다)
		bus.set_register(6, ADDR_STATUS, STATUS_OVERHEAT)
		result = driver.call_arm_torque(True)
		assert not result.success
		assert 'arm 6 (arm_gripper) reports overheat' in result.message
		assert bus.writes[mark:] == []
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in ARM_IDS)
		spin_for(ros_node, 0.3)
		assert driver.arm_ready[-1:] == [True], driver.arm_ready

		# 팔 하나의 토크가 꺼져 다시 켤 때도, 켜져 있는 그리퍼의 과부하로 거절하지 않는다
		bus.set_register(6, ADDR_STATUS, STATUS_OVERLOAD)
		bus.set_register(2, ADDR_TORQUE, 0)
		mark = len(bus.writes)
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert [(write[1], write[2]) for write in bus.writes[mark:]] == [
			(2, ADDR_GOAL_POSITION), (2, ADDR_TORQUE)]
	finally:
		driver.stop()


def test_arm_enable_retries_a_failed_id_once(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		bus.fail_next_writes(4, ADDR_TORQUE, 1)
		result = driver.call_arm_torque(True)
		assert result.success, result.message
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in ARM_IDS)
		# 4번만 한 번 더 (목표 = 지금 위치 → 토크)
		assert [(write[1], write[2]) for write in driver.writes_after_startup()] == (
			[(motor_id, ADDR_GOAL_POSITION) for motor_id in ARM_IDS] +
			[(motor_id, ADDR_TORQUE) for motor_id in ARM_IDS] +
			[(4, ADDR_GOAL_POSITION), (4, ADDR_TORQUE)])
		assert wait_until(ros_node, lambda: driver.arm_ready[-1:] == [True], 2.0), driver.arm_ready
	finally:
		driver.stop()


def test_arm_enable_failure_rolls_back_only_motors_that_were_off(ros_node, bus):
	# 1 · 2번은 다른 프로그램이 켜 둔 토크로 팔을 잡고 있다. 3번은 토크 켜기에 두 번(처음 + 다시 시도) 응답하지 않는다.
	place_arm(bus)
	for motor_id in (1, 2):
		bus.set_register(motor_id, ADDR_TORQUE, 1)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		bus.fail_next_writes(3, ADDR_TORQUE, 2)
		result = driver.call_arm_torque(True)
		assert not result.success
		assert 'no acknowledgement from arm ids [3]' in result.message
		writes = torque_writes(driver.writes_after_startup())
		assert writes.count((3, 1)) == 2
		# 원래 꺼져 있던 3 ~ 6번만 다시 끈다. 잡고 있던 1 · 2번에는 끄기를 보내지 않는다
		assert [entry for entry in writes if entry[1] == 0] == [(3, 0), (4, 0), (5, 0), (6, 0)]
		assert [bus.register(motor_id, ADDR_TORQUE) for motor_id in ARM_IDS] == [1, 1, 0, 0, 0, 0]
		# 명령은 받지 않는다
		mark = len(bus.writes)
		driver.arm_stream(ARM_IDS, [ARM_PRESENT[motor_id] for motor_id in ARM_IDS], 0.3)
		spin_for(ros_node, 0.2)
		assert bus.writes[mark:] == []
		assert wait_until(ros_node, lambda: driver.arm_ready == [False], 2.0), driver.arm_ready
	finally:
		driver.stop()


def test_arm_torque_off_failure_is_retried_on_slow_cycle(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		bus.fail_next_writes(4, ADDR_TORQUE, 2)
		mark = len(bus.writes)
		result = driver.call_arm_torque(False)
		assert not result.success
		assert 'no acknowledgement from arm ids [4]' in result.message
		assert bus.register(4, ADDR_TORQUE) == 1
		# 끄기를 확인할 때까지 다시 켜기를 거부한다 (아무것도 쓰지 않는다)
		refused = driver.call_arm_torque(True)
		assert not refused.success
		assert 'not confirmed' in refused.message
		# 느린 주기(1 s)마다 4번에만 다시 보낸다: 한 번 더 실패 → 다음에 꺼진다
		assert wait_until(ros_node, lambda: bus.register(4, ADDR_TORQUE) == 0, 4.0)
		assert bus.writes[mark + len(ARM_IDS):] == [(0x03, 4, ADDR_TORQUE, b'\x00')] * 2
		assert wait_until(ros_node, lambda: driver.arm_ready[-1:] == [False], 2.0), driver.arm_ready
		assert driver.call_arm_torque(True).success
	finally:
		output = driver.stop()
	assert 'arm torque disable confirmed for ids [4]' in output


@pytest.mark.parametrize('fault', ['unresponsive', 'overheat'])
def test_arm_enable_refused_when_a_motor_is_unresponsive_or_faulted(ros_node, bus, fault):
	place_arm(bus)
	if fault == 'unresponsive':
		bus.set_responsive(2, False)
	else:
		bus.set_register(5, ADDR_STATUS, STATUS_OVERHEAT)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		result = driver.call_arm_torque(True)
		assert not result.success
		assert ('arm 2 (arm_shoulder_lift) is not responding' if fault == 'unresponsive' else 'overheat') in (
			result.message)
		assert driver.writes_after_startup() == []
		assert wait_until(ros_node, lambda: driver.arm_ready == [False], 2.0), driver.arm_ready
	finally:
		driver.stop()


def test_arm_command_dropped_when_a_commanded_motor_was_not_read(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		# 4번이 응답하지 않으면 4번이 든 명령은 통째로 버린다 (읽지 못한 위치 0 기준으로 자르지 않는다)
		bus.set_responsive(4, False)
		spin_for(ros_node, 0.3)
		mark = len(bus.writes)
		driver.arm_stream(ARM_IDS, [ARM_PRESENT[motor_id] + 50 for motor_id in ARM_IDS], 0.5)
		assert goal_position_writes(bus.writes[mark:]) == []
		# 4번이 없는 명령은 쓴다
		driver.arm_stream([1], [1150], 0.3)
		assert wait_until(ros_node, lambda: bus.goal_position(1) == 1150, 1.0)
	finally:
		output = driver.stop()
	assert 'present position was not read this cycle' in output


def test_arm_holds_without_writes_after_commands_stop(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert driver.call_arm_torque(True).success
		goals = [ARM_PRESENT[motor_id] + 50 for motor_id in ARM_IDS]
		driver.arm_stream(ARM_IDS, goals, 0.5)
		spin_for(ros_node, 0.2)
		# 명령이 끊긴 뒤 3 s 넘게 드라이버는 아무것도 쓰지 않는다 (토크 · 마지막 목표 유지)
		mark = len(bus.writes)
		spin_for(ros_node, 3.5)
		assert bus.writes[mark:] == []
		assert [bus.goal_position(motor_id) for motor_id in ARM_IDS] == goals
		assert all(bus.register(motor_id, ADDR_TORQUE) == 1 for motor_id in ARM_IDS)
		assert wait_until(ros_node, lambda: driver.arm_ready[-1:] == [True], 2.0), driver.arm_ready
	finally:
		driver.stop()


def test_arm_ids_overlapping_wheel_ids_rejected_only_with_arm_write(ros_node, bus):
	driver = DriverUnderTest(
		ros_node, bus, is_arm_write_enabled=True, params=['arm_ids:=[1,2,3,4,5,7]'])
	try:
		assert driver.process.wait(timeout=10.0) != 0
	finally:
		output = driver.stop()
	assert 'arm_ids must not overlap wheel_ids' in output
	# 팔 쓰기를 끄면 arm_ids를 보지 않는다
	driver = DriverUnderTest(
		ros_node, bus, is_arm_write_enabled=False, params=['arm_ids:=[1,2,3,4,5,7]'])
	try:
		assert driver.wait_for_states()
		assert driver.process.poll() is None
	finally:
		driver.stop()


def test_arm_ready_follows_torque_service(ros_node, bus):
	place_arm(bus)
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		# 시작할 때 한 번 false
		assert wait_until(ros_node, lambda: driver.arm_ready == [False], 5.0)
		assert driver.call_arm_torque(True).success
		assert wait_until(ros_node, lambda: driver.arm_ready == [False, True], 2.0)
		assert driver.call_arm_torque(True).success   # 이미 켜짐: 바뀌지 않으므로 다시 발행하지 않는다
		assert driver.call_arm_torque(False).success
		assert wait_until(ros_node, lambda: driver.arm_ready == [False, True, False], 2.0)
		assert driver.call_arm_torque(True).success
		assert wait_until(ros_node, lambda: driver.arm_ready == [False, True, False, True], 2.0)
		# 켜기가 실패하면 (여기서는 팔 하나가 응답하지 않음) false
		bus.set_responsive(3, False)
		assert not driver.call_arm_torque(True).success
		assert wait_until(ros_node, lambda: driver.arm_ready == [False, True, False, True, False], 2.0)
	finally:
		driver.stop()


def test_arm_registers_report_eeprom_values(ros_node, bus):
	for motor_id in ARM_IDS:
		bus.set_word(motor_id, ADDR_MIN_POSITION_LIMIT, 500 + motor_id)
		bus.set_word(motor_id, ADDR_MAX_POSITION_LIMIT, 3500 + motor_id)
		bus.set_register(motor_id, ADDR_P_COEFFICIENT, 16)
		bus.set_register(motor_id, ADDR_D_COEFFICIENT, 32)
		bus.set_register(motor_id, ADDR_I_COEFFICIENT, motor_id)
		bus.set_word(motor_id, ADDR_HOMING_OFFSET, 100 * motor_id)
	bus.set_word(2, ADDR_HOMING_OFFSET, 0x800 | 1234)   # 부호-크기 bit 11: -1234 (LeRobot 보정 JSON 값)
	bus.set_responsive(6, False)   # 그리퍼는 처음에 응답하지 않는다
	driver = DriverUnderTest(ros_node, bus, is_arm_write_enabled=True)
	try:
		assert driver.wait_for_states()
		assert wait_until(ros_node, lambda: len(driver.arm_registers) == 1, 5.0)
		first = driver.arm_registers[0]
		assert list(first.ids) == ARM_IDS
		assert list(first.valid) == [True] * 5 + [False]
		assert list(first.homing_offset) == [100, -1234, 300, 400, 500, 0]
		assert list(first.min_position_limit) == [501, 502, 503, 504, 505, 0]
		assert list(first.max_position_limit) == [3501, 3502, 3503, 3504, 3505, 0]
		assert list(first.p_coefficient) == [16] * 5 + [0]
		assert list(first.d_coefficient) == [32] * 5 + [0]
		assert list(first.i_coefficient) == [1, 2, 3, 4, 5, 0]
		# 그리퍼가 응답하면 다시 읽어 발행한다
		bus.set_responsive(6, True)
		assert wait_until(ros_node, lambda: len(driver.arm_registers) == 2, 5.0)
		second = driver.arm_registers[1]
		assert list(second.valid) == [True] * 6
		assert list(second.homing_offset) == [100, -1234, 300, 400, 500, 600]
		assert second.max_position_limit[5] == 3506
		# 읽기만 한다
		assert bus.written_ids().isdisjoint(ARM_IDS)
	finally:
		driver.stop()


# ---------------------------------------------------------------------------
# 주기 예산: 응답이 늦은 버스에서 30 Hz
# ---------------------------------------------------------------------------

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'lekiwi_motor_driver.yaml'
# 응답마다 늦추는 시간 [s]. 실물에서 재지 않았다: USB 왕복(1 ms 프레임) + 1 Mbps 전송(READ 한 번 약 0.3 ms)을 넉넉히 잡았다
REPLY_DELAY_S = 0.002
ADDR_CONFIG_BLOCK = ADDR_MODE   # 운전 모드 · 토크 읽기 (33~40)


class BusProcess:
	"""FakeStsBus를 따로 프로세스로 띄운다. 응답 시각이 시험 프로세스의 rclpy 스레드(GIL)에 흔들리지 않는다."""

	def __init__(self, reply_delay: float) -> None:
		self.process = subprocess.Popen(
			[sys.executable, str(Path(__file__).resolve().parent / 'fake_sts_bus.py'), str(reply_delay)],
			stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
		self.port = self.process.stdout.readline().strip()

	def close(self) -> dict[str, list[list[float]]]:
		"""버스를 끝내고 {'reads': [(시각, id, 주소)], 'writes': [(명령, id, 주소)]}를 돌려준다."""
		output, _ = self.process.communicate(timeout=10.0)
		return json.loads(output)


def read_cycles(reads: list[list[float]], start: float, end: float) -> list[list[list[float]]]:
	"""start ~ end의 READ를 주기마다 묶는다. 주기는 모터 1의 상태 읽기(56)로 시작한다 (slots_ 순서)."""
	cycles = []
	for read in reads:
		if not start <= read[0] <= end:
			continue
		if (read[1], read[2]) == (ARM_IDS[0], ADDR_POSITION) or not cycles:
			cycles.append([])
		cycles[-1].append(read)
	return cycles[1:-1]   # 창 끝에 걸린 주기는 뺀다


def test_states_at_30_hz_within_cycle_budget_while_commands_stream(ros_node):
	"""응답마다 REPLY_DELAY_S가 걸려도 states는 약 30 Hz이고, 주기마다 상태 9개 + 느린 읽기 많아야 1개만 읽는다.

	주기당 READ 수가 판정 기준이다. 시간(바쁜 시간 · 주기율)은 CPU 부하에 흔들리므로 느슨하게만 본다.
	"""
	params = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))['lekiwi_motor_driver_node']['ros__parameters']
	rate = params['publish_rate_hz']
	assert rate == 30.0
	period = 1.0 / rate
	bus = BusProcess(REPLY_DELAY_S)
	driver = DriverUnderTest(
		ros_node, bus, is_arm_write_enabled=True,
		params=[f'publish_rate_hz:={rate}', f'read_timeout_ms:={params["read_timeout_ms"]}'])
	try:
		assert wait_until(ros_node, lambda: len(driver.states) > 0, 10.0)
		assert driver.call_torque(True).success
		assert driver.call_arm_torque(True).success
		# 베이스 컨트롤러(50 Hz) · 팔로워 팔(30 Hz)처럼 명령을 보내며 느린 읽기가 5번 넘게 돌도록 잰다
		mark = len(driver.states)
		started = time.monotonic()
		next_wheel = next_arm = started
		arm_ticks = 2048
		published = {ADDR_GOAL_SPEED: 0, ADDR_GOAL_POSITION: 0}
		while time.monotonic() < started + 5.5:
			now = time.monotonic()
			if now >= next_wheel:
				driver.command([8, 9, 7], [1.0, -1.0, 0.5])
				published[ADDR_GOAL_SPEED] += 1
				next_wheel += 0.02
			if now >= next_arm:
				arm_ticks = 2058 if arm_ticks == 2048 else 2048   # 가상 팔은 움직이지 않는다: 1024 tick 제한 안
				driver.arm_command(ARM_IDS, [arm_ticks] * len(ARM_IDS))
				published[ADDR_GOAL_POSITION] += 1
				next_arm += period
			rclpy.spin_once(ros_node, timeout_sec=0.002)
		ended = time.monotonic()
		states = driver.states[mark:]
	finally:
		output = driver.stop()
		summary = bus.close()

	stamps = [state.header.stamp.sec + state.header.stamp.nanosec * 1e-9 for state in states]
	states_rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
	# 버스 쪽에서 주기마다 (모터 1 상태 읽기 ~ 마지막 응답) 바쁜 시간과 주기 시작 간격
	cycles = read_cycles(summary['reads'], started, ended)
	busy_ms = [(cycle[-1][0] - cycle[0][0] + REPLY_DELAY_S) * 1000.0 for cycle in cycles]
	starts = [cycle[0][0] for cycle in cycles]
	gaps_ms = [(later - earlier) * 1000.0 for earlier, later in zip(starts, starts[1:])]
	busy_median = sorted(busy_ms)[len(busy_ms) // 2]
	print(
		f'reply delay {REPLY_DELAY_S * 1000:.1f} ms: states {states_rate:.2f} Hz, {len(cycles)} cycles, '
		f'busy median {busy_median:.1f} ms max {max(busy_ms):.1f} ms, '
		f'cycle start gap max {max(gaps_ms):.1f} ms (period {period * 1000:.1f} ms), '
		f'driver cycle warnings {output.count("reading motors took")}')

	# 시간은 느슨하게만 (CPU 부하에 흔들린다. 바쁜 시간은 가짜 버스 프로세스가 늦게 도는 만큼도 늘어난다):
	# 주기율 ± 20 % (20 Hz와는 구별된다), 바쁜 시간 가운데값 < 1.5 주기, 상태 95 %가 모두 online
	assert states_rate == pytest.approx(rate, rel=0.2)
	assert len(cycles) == pytest.approx(rate * (ended - started), rel=0.2)
	assert busy_median < 1.5 * period * 1000.0
	assert sum(all(motor.online for motor in state.motors) for state in states) >= 0.95 * len(states)
	# 판정: 주기마다 모터 9개 상태를 다 읽고, 느린 읽기(33)는 많아야 하나
	for cycle in cycles:
		assert sorted(motor_id for _, motor_id, address in cycle if address == ADDR_POSITION) == ARM_IDS + WHEEL_IDS
		assert sum(address == ADDR_CONFIG_BLOCK for _, _, address in cycle) <= 1
	# 모터마다 운전 모드 · 토크를 slow_read_period(1 s)마다 읽었다
	config_reads = [motor_id for cycle in cycles for _, motor_id, address in cycle if address == ADDR_CONFIG_BLOCK]
	assert all(config_reads.count(motor_id) >= 4 for motor_id in ARM_IDS + WHEEL_IDS), config_reads
	# 명령이 계속 버스에 갔다 (바퀴 · 팔 SYNC_WRITE). 깊이 1이라 읽는 동안 온 명령은 마지막 것만 쓴다 (부하가 크면 더 덮인다)
	sync_writes = [address for instruction, _, address in summary['writes'] if instruction == 0x83]
	assert all(sync_writes.count(address) >= 0.3 * count for address, count in published.items()), (
		published, {address: sync_writes.count(address) for address in published})


@pytest.mark.parametrize('outage_s', [0.13, 0.25])
def test_short_read_outage_comes_back_within_a_few_cycles(ros_node, bus, outage_s):
	"""팔 모터 하나의 짧은 응답 끊김은 1초 재시도로 늘어나지 않는다.

	130 ms: offline_after_failures(5번, 30 Hz 약 167 ms)에 못 미쳐 응답 없음으로 보지 않는다.
	250 ms: 응답 없음이 되지만 그 뒤 0.3 s는 주기마다 다시 읽어 바로 돌아온다.
	"""
	params = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))['lekiwi_motor_driver_node']['ros__parameters']
	driver = DriverUnderTest(ros_node, bus, params=[
		f'read_timeout_ms:={params["read_timeout_ms"]}',
		f'offline_after_failures:={params["offline_after_failures"]}'])
	try:
		assert driver.wait_for_states()
		spin_for(ros_node, 0.5)
		bus.set_responsive(3, False)
		off = time.time()   # states stamp(시스템 시각)과 비교한다
		while time.time() < off + outage_s:
			rclpy.spin_once(ros_node, timeout_sec=0.002)
		bus.set_responsive(3, True)
		on = time.time()
		spin_for(ros_node, 0.6)
	finally:
		output = driver.stop()

	def stamp(state: MotorStateArray) -> float:
		return state.header.stamp.sec + state.header.stamp.nanosec * 1e-9

	def elbow_online(state: MotorStateArray) -> bool:
		return next(motor.online for motor in state.motors if motor.id == 3)

	missed = [stamp(state) - off for state in driver.states if not elbow_online(state)]
	back = next(stamp(state) - on for state in driver.states if stamp(state) > on and elbow_online(state))
	print(f'outage {outage_s * 1000:.0f} ms: {len(missed)} states without the elbow, back {back * 1000:.0f} ms after the outage')
	assert missed, 'the outage did not land'
	assert back <= 0.2
	assert all(elbow_online(state) for state in driver.states if stamp(state) > on + back)
	if outage_s < 0.15:
		assert 'motor 3 (arm_elbow_flex) stopped responding' not in output
	else:
		assert 'motor 3 (arm_elbow_flex) stopped responding' in output
		assert 'motor 3 (arm_elbow_flex) is responding' in output
