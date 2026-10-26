"""lekiwi_zmq_host_node: lerobot LeKiwi 호스트 프로토콜(ZMQ)을 ROS 토픽 · 서비스로 바꾼다.

PC 는 ROS 없이 lerobot LeKiwiClient 로 붙는다 (명령 PULL 5555 · 관측 PUSH 5556, 둘 다 CONFLATE).
번역만 한다: 팔 단위 · 램프 · 이동 제한은 팔로워 팔, 바퀴 상한 · 끊김은 베이스 컨트롤러가 그대로 맡는다.
판단은 protocol · session · kinematics (ROS 없이 시험), 이 파일은 ROS · zmq 입출력만.
시스템 python 으로 돈다 (lerobot 을 import 하지 않는다).
"""

import json
import signal
import time
from typing import Any, Final

import rclpy
import zmq
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
	DurabilityPolicy,
	QoSProfile,
	ReliabilityPolicy,
	qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from zmq.utils.monitor import recv_monitor_message

from lekiwi_msgs.msg import LerobotState
from lekiwi_zmq_host.kinematics import BaseKinematics, lerobot_action_to_cmd_vel
from lekiwi_zmq_host.protocol import (
	ARM_JOINTS,
	STATE_TIMEOUT_S,
	build_observation,
	parse_command,
	state_problem,
)
from lekiwi_zmq_host.session import (
	CREATE_CMD_VEL,
	DESTROY_CMD_VEL,
	DISABLE_WHEELS,
	ENABLE_ARM,
	ENABLE_WHEELS,
	PUBLISH_ARM,
	PUBLISH_BASE,
	PUBLISH_ZERO,
	STATUS_CHANGED,
	Effect,
	Session,
)

NODE_NAME: Final[str] = 'lekiwi_zmq_host_node'
# 다른 노드 (src/docs/interfaces.md)
BASE_CONTROLLER_NODE: Final[str] = 'lekiwi_base_controller_node'
FOLLOWER_NODE: Final[str] = 'lekiwi_follower_arm_node'
DRIVER_NODE: Final[str] = 'lekiwi_motor_driver_node'
# 웹 게이트웨이: 조작 권한을 가진 브라우저가 있고 비상 정지가 걸려 있지 않은 동안만 있는 노드
OPERATOR_VIEW_NODE: Final[str] = 'lekiwi_web_operator_view'
NAVIGATION_CMD_VEL_NODE: Final[str] = 'collision_monitor'
# 토픽 · 서비스
STATE_TOPIC: Final[str] = 'lekiwi/lerobot/state'
CAMERA_TOPICS: Final[dict[str, str]] = {
	'front': 'lekiwi/sensor/front_camera/image_raw/compressed',
	'wrist': 'lekiwi/sensor/wrist_camera/image_raw/compressed',
}
ARM_STATUS_TOPIC: Final[str] = 'lekiwi/arm/status'
MOTOR_READY_TOPIC: Final[str] = 'lekiwi/motor/ready'
ARM_READY_TOPIC: Final[str] = 'lekiwi/motor/arm_ready'
ESTOP_TOPIC: Final[str] = 'lekiwi/safety/estop_latched'
ARM_GOAL_TOPIC: Final[str] = 'lekiwi/arm/goal'
CMD_VEL_TOPIC: Final[str] = 'cmd_vel'
STATUS_TOPIC: Final[str] = 'lekiwi/zmq_host/status'
POWER_SERVICE: Final[str] = 'lekiwi/motor/power'
ARM_TORQUE_SERVICE: Final[str] = 'lekiwi/motor/arm_torque'
# 베이스 컨트롤러 파라미터 (lekiwi_base_controller_node 와 같은 이름) → BaseKinematics 인자 순서
BASE_PARAMETERS: Final[tuple[str, ...]] = (
	'wheel_radius', 'base_radius', 'kinematics_frame_x', 'kinematics_frame_y',
	'kinematics_frame_yaw', 'wheel_ids', 'wheel_directions')
STATUS_PERIOD_S: Final[float] = 1.0
# [s] 베이스 파라미터 요청에 이만큼 답이 없으면 (그사이 컨트롤러가 사라짐 등) 버리고 다시 묻는다
BASE_REQUEST_TIMEOUT_S: Final[float] = 3.0
# 드라이버 arm_ready · 베이스 컨트롤러 ready 와 같은 QoS: 늦게 붙어도 마지막 값을 받는다
LATCHED_QOS: Final[QoSProfile] = QoSProfile(
	depth=1,
	reliability=ReliabilityPolicy.RELIABLE,
	durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ZmqHostNode(Node):
	"""ZMQ 호스트 노드. 판단은 Session 이 한다 (이 클래스는 ROS · zmq 입출력만).

	구독: lekiwi/lerobot/state (LerobotState), 카메라 2대 (CompressedImage, ZMQ 클라이언트가 있을 때만),
		lekiwi/arm/status (String), lekiwi/motor/ready · arm_ready · lekiwi/safety/estop_latched (Bool)
	발행: lekiwi/arm/goal (JointState), cmd_vel (Twist, 세션이 active 일 때만),
		lekiwi/zmq_host/status (String, JSON)
	서비스 클라이언트: lekiwi/motor/power · lekiwi/motor/arm_torque (SetBool)
	"""

	def __init__(self, **kwargs: Any) -> None:
		"""파라미터 · zmq 소켓 · 구독 · 발행 · 타이머를 만든다 (kwargs 는 rclpy Node 로, 시험용)."""
		super().__init__(NODE_NAME, **kwargs)

		# ── 1) 파라미터 ───────────────────────────────────
		self.declare_parameter('port_zmq_cmd', 5555)
		self.declare_parameter('port_zmq_observations', 5556)
		self.declare_parameter('max_loop_freq_hz', 30.0)
		rate_hz = float(self.get_parameter('max_loop_freq_hz').value)
		if rate_hz <= 0.0:
			raise ValueError('max_loop_freq_hz must be positive')

		# ── 2) 내부 상태 ──────────────────────────────────
		self._session = Session()
		self._state: dict[str, Any] | None = None   # {'values', 'time'}
		self._state_problem: str | None = None
		self._frames: dict[str, tuple[Any, float]] = {}
		self._arm_status: dict[str, Any] = {}
		self._estop_latched: bool | None = None
		self._base: BaseKinematics | None = None
		self._base_reason = f'베이스 컨트롤러({BASE_CONTROLLER_NODE}) 파라미터를 아직 읽지 못했습니다'
		self._base_requested_at: float | None = None   # 답을 기다리는 요청을 보낸 때 (time.monotonic)
		self._client_count = 0
		self._is_peer_warned = False
		self._last_problem: str | None = None
		self._last_status: dict[str, Any] | None = None
		self._cmd_vel_pub: Any = None
		self._camera_subs: list[Any] = []

		# ── 3) ZMQ (lerobot LeKiwiHost 와 같은 소켓) ──────
		self._zmq = zmq.Context()
		self._cmd_socket = self._zmq.socket(zmq.PULL)
		self._obs_socket = self._zmq.socket(zmq.PUSH)
		for sock in (self._cmd_socket, self._obs_socket):
			sock.setsockopt(zmq.CONFLATE, 1)
			sock.setsockopt(zmq.LINGER, 0)
		self._cmd_socket.bind(f'tcp://*:{self.get_parameter("port_zmq_cmd").value}')
		self._obs_socket.bind(f'tcp://*:{self.get_parameter("port_zmq_observations").value}')
		# 관측을 받는 클라이언트 수 (카메라 구독을 그때만 둔다)
		self._monitor = self._obs_socket.get_monitor_socket(
			zmq.EVENT_ACCEPTED | zmq.EVENT_DISCONNECTED)

		# ── 4) Subscriber / Publisher / Client ────────────
		# 팔로워 발행과 같은 QoS (reliable, 깊이 10)
		self.create_subscription(LerobotState, STATE_TOPIC, self.state_callback, 10)
		self.create_subscription(String, ARM_STATUS_TOPIC, self.arm_status_callback, 10)
		self.create_subscription(Bool, MOTOR_READY_TOPIC, self.motor_ready_callback, LATCHED_QOS)
		self.create_subscription(Bool, ARM_READY_TOPIC, self.arm_ready_callback, LATCHED_QOS)
		self.create_subscription(Bool, ESTOP_TOPIC, self.estop_callback, LATCHED_QOS)
		# 팔로워 구독과 같은 QoS (게이트웨이도 sensor_data 로 낸다)
		self._goal_pub = self.create_publisher(JointState, ARM_GOAL_TOPIC, qos_profile_sensor_data)
		self._status_pub = self.create_publisher(String, STATUS_TOPIC, LATCHED_QOS)
		self._power_client = self.create_client(SetBool, POWER_SERVICE)
		self._arm_torque_client = self.create_client(SetBool, ARM_TORQUE_SERVICE)
		self._parameter_client = AsyncParameterClient(self, BASE_CONTROLLER_NODE)

		# ── 5) Timer ──────────────────────────────────────
		self.create_timer(1.0 / rate_hz, self.timer_callback)
		self.create_timer(STATUS_PERIOD_S, self.status_timer_callback)
		self.get_logger().info(
			f'ZMQ 명령 :{self.get_parameter("port_zmq_cmd").value} · '
			f'관측 :{self.get_parameter("port_zmq_observations").value}')

	# ── 구독 콜백 ─────────────────────────────────────────
	def state_callback(self, msg: LerobotState) -> None:
		"""lerobot 관측 상태 9개. 이름 · 값이 맞지 않으면 쓰지 않는다."""
		names, values = list(msg.names), list(msg.values)
		self._state_problem = state_problem(names, values)
		if self._state_problem is None:
			self._state = {'values': dict(zip(names, values)), 'time': time.monotonic()}

	def camera_callback(self, name: str, msg: CompressedImage) -> None:
		"""카메라 JPEG 최신 하나 (다시 압축하지 않는다)."""
		self._frames[name] = (msg.data, time.monotonic())

	def arm_status_callback(self, msg: String) -> None:
		try:
			status = json.loads(msg.data)
		except ValueError:
			return
		if isinstance(status, dict):
			self._arm_status = status

	def motor_ready_callback(self, msg: Bool) -> None:
		self._run(self._session.on_external(time.monotonic(), msg.data, None))

	def arm_ready_callback(self, msg: Bool) -> None:
		self._run(self._session.on_external(time.monotonic(), None, msg.data))

	def estop_callback(self, msg: Bool) -> None:
		self._estop_latched = msg.data
		self._run(self._session.on_estop(time.monotonic(), msg.data))

	# ── 타이머 ────────────────────────────────────────────
	def timer_callback(self) -> None:
		"""max_loop_freq_hz: 접속 수 → 명령 → 끊김 → 관측 → 상태."""
		now = time.monotonic()
		self._read_monitor()
		self._read_command(now)
		self._run(self._session.on_tick(now))
		meta = {
			**self._session.status(),
			'reason': self._reason(now),
			'follower_state': self._arm_status.get('state'),
		}
		observation = build_observation(self._state, self._frames, meta, now)
		if observation is not None:
			try:
				self._obs_socket.send_string(observation, flags=zmq.NOBLOCK)
			except zmq.Again:
				pass   # 받는 클라이언트가 없다
		status = self._status(now)
		if status != self._last_status:
			self._publish_status(status)

	def status_timer_callback(self) -> None:
		"""1 Hz: 베이스 파라미터를 (다시) 읽고 상태를 다시 낸다."""
		self._refresh_base_parameters()
		self._publish_status(self._status(time.monotonic()))

	# ── ZMQ ───────────────────────────────────────────────
	def _read_monitor(self) -> None:
		while True:
			try:
				event = recv_monitor_message(self._monitor, zmq.NOBLOCK)['event']
			except zmq.Again:
				break
			if event == zmq.EVENT_ACCEPTED:
				self._client_count += 1
			elif event == zmq.EVENT_DISCONNECTED:
				self._client_count = max(0, self._client_count - 1)
		self._sync_camera_subscriptions()

	def _sync_camera_subscriptions(self) -> None:
		if self._client_count > 0 and not self._camera_subs:
			self._camera_subs = [
				self.create_subscription(
					CompressedImage, topic,
					lambda msg, name=name: self.camera_callback(name, msg),
					qos_profile_sensor_data)
				for name, topic in CAMERA_TOPICS.items()]
		elif self._client_count == 0 and self._camera_subs:
			for sub in self._camera_subs:
				self.destroy_subscription(sub)
			self._camera_subs = []
			self._frames.clear()

	def _read_command(self, now: float) -> None:
		try:
			frame = self._cmd_socket.recv(flags=zmq.NOBLOCK, copy=False)
		except zmq.Again:
			return
		try:
			self._handle_command(now, frame)
		except Exception as error:   # noqa: BLE001 — 인증 없는 네트워크 입력 하나로 노드가 죽지 않게 (바퀴 토크가 켜진 채 남는다)
			self.get_logger().error(f'명령을 처리하지 못해 버립니다: {error!r}')

	def _handle_command(self, now: float, frame: Any) -> None:
		try:
			text = frame.bytes.decode('utf-8')
		except UnicodeDecodeError:
			text = ''
		cmd = parse_command(text)
		if cmd.problems and cmd.problems[0] != self._last_problem:
			self.get_logger().warning(cmd.problems[0])
		self._last_problem = cmd.problems[0] if cmd.problems else None
		self._run(self._session.on_command(now, cmd, self._start_blockers, self._peer_address(frame)))

	def _peer_address(self, frame: Any) -> str | None:
		"""명령을 보낸 주소. pyzmq · libzmq 가 주지 않으면 None (세션 주인 검사 없이 돈다)."""
		try:
			return frame.get('Peer-Address')
		except (zmq.ZMQError, AttributeError) as error:
			if not self._is_peer_warned:
				self._is_peer_warned = True
				self.get_logger().warning(f'명령을 보낸 주소를 알 수 없어 세션 주인을 검사하지 않습니다: {error!r}')
			return None

	# ── 세션 ──────────────────────────────────────────────
	def _start_blockers(self) -> list[str]:
		"""세션 시작을 막는 이유 (우선순위 순)."""
		names = {name for name, _namespace in self.get_node_names_and_namespaces()}
		own = (self.get_name(), self.get_namespace())
		others = sorted(
			f'{info.node_namespace.rstrip("/")}/{info.node_name}'
			for info in self.get_publishers_info_by_topic(CMD_VEL_TOPIC)
			if (info.node_name, info.node_namespace) != own)
		reasons = []
		if OPERATOR_VIEW_NODE in names:
			reasons.append('웹 화면이 조작 중입니다')
		if any(other.endswith(f'/{NAVIGATION_CMD_VEL_NODE}') for other in others):
			reasons.append('자율 주행 중입니다 (collision_monitor 가 cmd_vel 을 보냅니다)')
		elif others:
			reasons.append(f'다른 프로그램이 cmd_vel 을 보내고 있습니다: {", ".join(others)}')
		if self._estop_latched and self.count_publishers(ESTOP_TOPIC) > 0:
			reasons.append('비상 정지가 걸려 있습니다')
		if FOLLOWER_NODE not in names:
			reasons.append(f'팔로워 팔 노드({FOLLOWER_NODE})가 없습니다')
		if DRIVER_NODE not in names:
			reasons.append(f'모터 드라이버({DRIVER_NODE})가 없습니다')
		if self._base is None:
			reasons.append(self._base_reason)
		return reasons

	def _run(self, effects: list[Effect]) -> None:
		"""Session 이 돌려준 일을 순서대로 한다.

		일 하나가 세션을 바꾸면 (서비스가 없어 _call 이 바로 on_failure → blocked) 남은 일은 낡았으니 버린다.
		"""
		state = self._session.state
		for effect in effects:
			if self._session.state != state:
				break
			if effect.kind == ENABLE_WHEELS:
				self._call(self._power_client, True, f'바퀴 토크({POWER_SERVICE})')
			elif effect.kind == ENABLE_ARM:
				self._call(self._arm_torque_client, True, f'팔 토크({ARM_TORQUE_SERVICE})')
			elif effect.kind == DISABLE_WHEELS:
				self._call(self._power_client, False, f'바퀴 토크({POWER_SERVICE})')
			elif effect.kind == CREATE_CMD_VEL and self._cmd_vel_pub is None:
				self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
			elif effect.kind == DESTROY_CMD_VEL and self._cmd_vel_pub is not None:
				self.destroy_publisher(self._cmd_vel_pub)
				self._cmd_vel_pub = None
			elif effect.kind == PUBLISH_ZERO:
				self._publish_twist((0.0, 0.0, 0.0))
			elif effect.kind == PUBLISH_BASE:
				# 세션 중 베이스 컨트롤러가 사라지면 (_refresh_base_parameters) 0 을 보낸다
				self._publish_twist(
					(0.0, 0.0, 0.0) if self._base is None
					else lerobot_action_to_cmd_vel(*effect.value, self._base))
			elif effect.kind == PUBLISH_ARM:
				goal = JointState()
				goal.header.stamp = self.get_clock().now().to_msg()
				goal.name = list(ARM_JOINTS)
				goal.position = [effect.value[name] for name in ARM_JOINTS]
				self._goal_pub.publish(goal)
			elif effect.kind == STATUS_CHANGED:
				status = self._status(time.monotonic())
				self.get_logger().info(
					f'세션 {status["session"]}'
					+ (f': {status["reason"]}' if status['reason'] else '')
					+ (f' (클라이언트 {status["client_ip"]})' if status['client_ip'] else ''))
				self._publish_status(status)

	def _publish_twist(self, twist: tuple[float, float, float]) -> None:
		if self._cmd_vel_pub is None:
			return
		msg = Twist()
		msg.linear.x, msg.linear.y, msg.angular.z = twist
		self._cmd_vel_pub.publish(msg)

	def _call(self, client: Any, is_enabled: bool, label: str) -> None:
		"""SetBool 비동기 호출. 켜기가 실패하면 세션을 blocked 로."""
		if not client.service_is_ready():
			self._service_failed(is_enabled, f'{label} 서비스가 없습니다')
			return
		future = client.call_async(SetBool.Request(data=is_enabled))
		future.add_done_callback(
			lambda done: self._service_done(done, is_enabled, label))

	def _service_done(self, future: Any, is_enabled: bool, label: str) -> None:
		response = future.result() if future.exception() is None else None
		if response is None:
			self._service_failed(is_enabled, f'{label} 호출 실패: {future.exception()}')
		elif not response.success:
			self._service_failed(is_enabled, f'{label} 실패: {response.message}')

	def _service_failed(self, is_enabled: bool, reason: str) -> None:
		self.get_logger().warning(reason)
		if is_enabled:
			self._run(self._session.on_failure(time.monotonic(), reason))

	# ── 베이스 파라미터 ──────────────────────────────────
	def _refresh_base_parameters(self) -> None:
		"""베이스 컨트롤러가 없으면 잊고, 있고 아직 모르면 읽는다 (다시 켜진 컨트롤러의 값을 쓰게)."""
		names = {name for name, _namespace in self.get_node_names_and_namespaces()}
		if BASE_CONTROLLER_NODE not in names:
			self._base = None
			self._base_requested_at = None   # 사라진 컨트롤러에 보낸 요청은 답이 오지 않을 수 있다
			self._base_reason = f'베이스 컨트롤러({BASE_CONTROLLER_NODE})가 없습니다'
			return
		now = time.monotonic()
		is_pending = (
			self._base_requested_at is not None
			and now - self._base_requested_at < BASE_REQUEST_TIMEOUT_S)
		if self._base is not None or is_pending:
			return
		if not self._parameter_client.services_are_ready():
			return
		self._base_requested_at = now
		self._parameter_client.get_parameters(
			list(BASE_PARAMETERS), callback=self._base_parameters_callback)

	def _base_parameters_callback(self, future: Any) -> None:
		self._base_requested_at = None
		response = future.result() if future.exception() is None else None
		if response is None or len(response.values) != len(BASE_PARAMETERS):
			self._base_reason = '베이스 컨트롤러 파라미터를 읽지 못했습니다'
			return
		values = [parameter_value_to_python(value) for value in response.values]
		try:
			self._base = BaseKinematics(*values)
		except (TypeError, ValueError) as error:
			self._base_reason = f'베이스 컨트롤러 파라미터를 쓸 수 없습니다: {error}'
			self.get_logger().error(self._base_reason)
			return
		self.get_logger().info(
			'베이스 기구학: ' + ', '.join(f'{n}={v}' for n, v in zip(BASE_PARAMETERS, values)))

	# ── 상태 ──────────────────────────────────────────────
	def _reason(self, now: float) -> str | None:
		"""세션 이유, 없으면 관측을 보내지 못하는 이유."""
		reason = self._session.status()['reason']
		if reason:
			return reason
		if self._state_problem:
			return self._state_problem
		if self._state is not None and now - self._state['time'] <= STATE_TIMEOUT_S:
			return None
		follower = self._arm_status.get('reason')
		return f'관측 상태({STATE_TOPIC})가 없습니다' + (f' — 팔로워: {follower}' if follower else '')

	def _status(self, now: float) -> dict[str, Any]:
		return {
			**self._session.status(),
			'reason': self._reason(now),
			'clients': self._client_count,
			'client_ip': self._session.owner,
			'estop_unknown': self._estop_latched is None or self.count_publishers(ESTOP_TOPIC) == 0,
		}

	def _publish_status(self, status: dict[str, Any]) -> None:
		self._last_status = status
		self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

	def destroy_node(self) -> None:
		"""cmd_vel 0 을 한 번 보내고 소켓을 닫는다 (바퀴 토크는 드라이버가 3 s 뒤 끈다)."""
		self._publish_twist((0.0, 0.0, 0.0))
		for sock in (self._monitor, self._cmd_socket, self._obs_socket):
			sock.close(linger=0)
		self._zmq.term()
		super().destroy_node()


def main(args: list[str] | None = None) -> None:
	"""노드 진입점."""
	rclpy.init(args=args)
	node = None
	try:
		node = ZmqHostNode()
		rclpy.spin(node)
	except (KeyboardInterrupt, ExternalShutdownException):
		pass   # Ctrl+C · ros2 launch 종료 (Jazzy: ExternalShutdownException)
	finally:
		# ros2 launch 가 SIGINT 를 한 번 더 보낸다: 정리하는 동안 끊기지 않게
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		if node is not None:
			node.destroy_node()
		rclpy.try_shutdown()


if __name__ == '__main__':
	main()
