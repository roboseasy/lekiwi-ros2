"""lekiwi_follower_arm_node: 리더 관절 목표를 팔로워 팔 tick 명령으로 바꿔 보낸다.

lerobot 보정(<calibration_dir>/<robot_id>.json)으로 LeRobot 단위 ↔ tick을 바꾸고
입력 검사 · 시작 램프 · 주기당 이동 제한 · 목표 끊김 유지를 한다 (follower_logic).
모터 버스는 열지 않는다. 버스 주인은 lekiwi_motor_driver_node다.
lerobot · numpy 2.2가 든 python으로 실행한다 (launch의 prefix, README).
"""

import json
import signal
import socket
import time
from pathlib import Path
from typing import Final

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
	DurabilityPolicy,
	QoSProfile,
	ReliabilityPolicy,
	qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from lekiwi_lerobot.follower_logic import (
	ARM_IDS,
	ARM_JOINTS,
	LEROBOT_STATE_NAMES,
	STATE_ERROR,
	ArmConverter,
	FollowerArmLogic,
	FollowerController,
	base_velocity,
	make_status,
)
from lekiwi_msgs.msg import ArmCommand, ArmRegisters, LerobotState, MotorStateArray

NODE_NAME: Final[str] = 'lekiwi_follower_arm_node'
DEFAULT_CALIBRATION_DIR: Final[str] = (
	'~/.cache/huggingface/lerobot/calibration/robots/lekiwi')
# FollowerController 인자와 같은 이름의 파라미터
CONTROLLER_PARAMETERS: Final[tuple[str, ...]] = (
	'ramp_threshold_deg', 'ramp_time_s', 'max_step_deg', 'goal_timeout_s',
	'track_tolerance_deg', 'track_timeout_s', 'rest_time_s', 'rest_tolerance_deg', 'follow_step_deg')
# 드라이버의 arm_ready · arm_registers와 같은 QoS: 늦게 붙어도 마지막 값을 받는다
LATCHED_QOS: Final[QoSProfile] = QoSProfile(
	depth=1,
	reliability=ReliabilityPolicy.RELIABLE,
	durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FollowerArmNode(Node):
	"""팔로워 팔 노드. 판단은 FollowerArmLogic이 한다 (이 클래스는 ROS 입출력만).

	구독: lekiwi/motor/states (MotorStateArray), lekiwi/motor/arm_ready (Bool),
		lekiwi/motor/arm_registers (ArmRegisters), lekiwi/arm/goal (JointState)
	발행: lekiwi/arm/state (JointState), lekiwi/motor/arm_command (ArmCommand),
		lekiwi/arm/status (String, JSON), lekiwi/lerobot/state (LerobotState)
	서비스: lekiwi/arm/rest (Trigger, 접은 자세로 천천히 돌아간다)
	"""

	def __init__(self) -> None:
		"""파라미터 · 보정 · 구독 · 발행 · 타이머를 만든다."""
		super().__init__(NODE_NAME)

		# ── 1) 파라미터 ───────────────────────────────────
		self.declare_parameter('robot_id', socket.gethostname())
		self.declare_parameter('calibration_dir', DEFAULT_CALIBRATION_DIR)
		self.declare_parameter('rate_hz', 30.0)
		self.declare_parameter('ramp_threshold_deg', 8.0)
		self.declare_parameter('ramp_time_s', 2.0)
		self.declare_parameter('max_step_deg', 10.0)
		self.declare_parameter('follow_step_deg', 45.0)
		self.declare_parameter('goal_timeout_s', 0.5)
		self.declare_parameter('track_tolerance_deg', 4.0)
		self.declare_parameter('track_timeout_s', 0.3)
		self.declare_parameter('state_timeout_s', 0.5)
		# 기본은 없음 (로봇마다 다르다: robot_profile.yaml에 둔다). 타입이 틀린 값도 받아 오류 상태로 알린다
		self.declare_parameter(
			'rest_pose_ticks', None, ParameterDescriptor(dynamic_typing=True))
		self.declare_parameter('rest_time_s', 3.0)
		self.declare_parameter('rest_tolerance_deg', 3.0)
		self.declare_parameter('motor_states_topic', 'lekiwi/motor/states')
		self.declare_parameter('arm_ready_topic', 'lekiwi/motor/arm_ready')
		self.declare_parameter(
			'arm_registers_topic', 'lekiwi/motor/arm_registers')
		self.declare_parameter('goal_topic', 'lekiwi/arm/goal')
		self.declare_parameter('state_topic', 'lekiwi/arm/state')
		self.declare_parameter('command_topic', 'lekiwi/motor/arm_command')
		self.declare_parameter('status_topic', 'lekiwi/arm/status')
		self.declare_parameter('lerobot_state_topic', 'lekiwi/lerobot/state')
		self.declare_parameter('rest_service', 'lekiwi/arm/rest')
		# 비우면 (설정 파일 · launch 기본값) 호스트 이름
		self._robot_id: str = (
			self.get_parameter('robot_id').value or socket.gethostname())
		rate_hz: float = self.get_parameter('rate_hz').value
		if rate_hz <= 0.0:
			raise ValueError('rate_hz must be positive')
		self._period_s = 1.0 / rate_hz

		# ── 2) 내부 상태 ──────────────────────────────────
		self._logic: FollowerArmLogic | None = None
		self._last_command_time = -1.0   # 마지막 arm_command를 낸 시각 (단조 시계)
		self._fault = self._load_calibration()   # None이 아니면 노드를 다시 켜야 한다

		# ── 3) Subscriber / Publisher ─────────────────────
		self.create_subscription(
			MotorStateArray,
			self.get_parameter('motor_states_topic').value,
			self.motor_states_callback,
			qos_profile_sensor_data,
		)
		self.create_subscription(
			Bool,
			self.get_parameter('arm_ready_topic').value,
			self.arm_ready_callback,
			LATCHED_QOS,
		)
		self.create_subscription(
			ArmRegisters,
			self.get_parameter('arm_registers_topic').value,
			self.arm_registers_callback,
			LATCHED_QOS,
		)
		# best effort 구독은 reliable · best effort 발행을 모두 받는다
		self.create_subscription(
			JointState,
			self.get_parameter('goal_topic').value,
			self.goal_callback,
			qos_profile_sensor_data,
		)
		self._state_pub = self.create_publisher(
			JointState, self.get_parameter('state_topic').value, 10)
		# 드라이버 바퀴 명령과 같은 QoS(1): 늦은 명령을 쌓지 않는다
		self._command_pub = self.create_publisher(
			ArmCommand, self.get_parameter('command_topic').value, 1)
		self._status_pub = self.create_publisher(
			String, self.get_parameter('status_topic').value, 10)
		self._lerobot_state_pub = self.create_publisher(
			LerobotState, self.get_parameter('lerobot_state_topic').value, 10)
		self.create_service(
			Trigger, self.get_parameter('rest_service').value, self.rest_callback)

		# ── 4) Timer ──────────────────────────────────────
		self.create_timer(1.0 / rate_hz, self.timer_callback)

	def _load_calibration(self) -> str | None:
		"""보정 파일을 읽어 변환기 · 판단을 만든다 (lerobot import · 판 확인 포함).

		Returns:
			실패 이유 (한국어). 성공하면 None.
		"""
		calibration_dir = Path(
			self.get_parameter('calibration_dir').value).expanduser()
		path = calibration_dir / f'{self._robot_id}.json'
		if not path.is_file():
			reason = f'보정 파일이 없습니다: {path}'
			self.get_logger().error(reason)
			return reason
		self.get_logger().info('lerobot · 보정을 불러옵니다 (Pi에서 약 5초)')
		started = time.monotonic()
		try:
			converter = ArmConverter(path)
		except Exception as error:  # lerobot 없음 · 판 다름 · 형식 오류: 오류 상태로 알린다
			reason = f'보정을 불러오지 못했습니다 ({path}): {error}'
			self.get_logger().error(reason)
			return reason
		try:
			controller = FollowerController(
				converter.limits,
				converter.units_per_degree,
				**{
					name: self.get_parameter(name).value
					for name in CONTROLLER_PARAMETERS},
			)
			# 설정이 없으면 None, 빈 문자열('')도 접은 자세 없음이다. rcl은 뒤 파일의 빈 목록([])을 무시해
			# 앞 파일 값이 남으므로, 게이트웨이의 arm_profile.yaml은 지울 때 ''를 쓴다
			rest_pose = self.get_parameter('rest_pose_ticks').value
			if rest_pose == '':
				rest_pose = None
			self._logic = FollowerArmLogic(
				converter, controller, self._robot_id,
				self.get_parameter('state_timeout_s').value,
				() if rest_pose is None else rest_pose)
		except ValueError as error:   # 파라미터 조합 오류: 죽지 않고 오류 상태로 알린다
			reason = f'팔로워 파라미터가 잘못됐습니다: {error}'
			self.get_logger().error(reason)
			return reason
		self.get_logger().info(
			f'lerobot · 보정 {path} 불러옴: {time.monotonic() - started:.1f} s')
		return None

	def motor_states_callback(self, msg: MotorStateArray) -> None:
		"""모터 상태 → 팔로워 자세 (LeRobot 단위) lekiwi/arm/state, lerobot 관측 상태 lekiwi/lerobot/state."""
		if self._logic is None:
			return
		follower = self._logic.on_motor_states(time.monotonic(), msg.motors)
		if follower is None:
			return
		state = JointState()
		state.header.stamp = msg.header.stamp
		state.name = list(ARM_JOINTS)
		state.position = [follower[name] for name in ARM_JOINTS]
		self._state_pub.publish(state)

		# 데이터셋용: 바퀴 3개도 이 샘플에서 응답했고, 팔 ID · 보정이 이 로봇의 모터와 맞을 때만
		base = base_velocity(msg.motors)
		if base is None or not self._logic.is_arm_confirmed:
			return
		lerobot_state = LerobotState()
		lerobot_state.header.stamp = msg.header.stamp
		lerobot_state.names = list(LEROBOT_STATE_NAMES)
		lerobot_state.values = [follower[name] for name in ARM_JOINTS] + list(base)
		self._lerobot_state_pub.publish(lerobot_state)

	def arm_ready_callback(self, msg: Bool) -> None:
		"""드라이버가 팔 명령을 받는가 (arm_torque true가 성공한 뒤에만 true)."""
		if self._logic is not None:
			self._logic.on_arm_ready(msg.data)

	def arm_registers_callback(self, msg: ArmRegisters) -> None:
		"""드라이버가 읽은 팔 모터 보정 레지스터: 보정 파일과 맞춰 보고 결과를 로그에 남긴다."""
		if self._logic is None:
			return
		error, warning = self._logic.on_registers(msg)
		if error is not None:
			self.get_logger().error(error)
		if warning is not None:
			self.get_logger().warning(warning)

	def goal_callback(self, msg: JointState) -> None:
		"""리더 목표. 팔 관절 6개가 모두 유한한 값으로 있어야 받는다."""
		if self._logic is None:
			return
		now = time.monotonic()
		if not self._logic.on_goal(now, msg.name, msg.position):
			self.get_logger().warning(
				'팔 관절 6개가 모두 있는 목표만 받습니다',
				throttle_duration_sec=5.0)
			return
		# 받자마자 명령을 낸다: 타이머를 기다리면 최대 1주기(33 ms) 늦는다
		self._step_and_command(now)

	def rest_callback(
		self, request: Trigger.Request, response: Trigger.Response,
	) -> Trigger.Response:
		"""lekiwi/arm/rest: 리더 목표를 무시하고 접은 자세로 천천히 돌아간다 (끝나면 status rested)."""
		del request
		reason = (
			f'팔로워를 쓸 수 없습니다: {self._fault}' if self._logic is None
			else self._logic.rest(time.monotonic()))
		response.success = reason is None
		response.message = reason or '접은 자세로 돌아갑니다'
		if response.success:
			self.get_logger().info(response.message)
		else:
			self.get_logger().warning(f'접은 자세 요청을 거절합니다: {reason}')
		return response

	def timer_callback(self) -> None:
		"""rate_hz: 입력을 검사하고 명령 · 상태를 낸다."""
		if self._logic is None:
			self._publish_status(
				make_status(STATE_ERROR, self._fault, self._robot_id))
			return
		now = time.monotonic()
		# 지난 한 주기 안에 목표가 와서 명령을 냈으면 건너뛴다 (같은 명령을 두 번 내지 않게).
		# 목표가 오지 않을 때의 램프 · 접은 자세 · 끊김 처리와 상태는 이 타이머가 맡는다
		if now - self._last_command_time >= self._period_s:
			self._step_and_command(now)
		self._publish_status(self._logic.status())

	def _step_and_command(self, now: float) -> None:
		ticks = self._logic.step(now)
		if ticks is None:
			return
		msg = ArmCommand()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.ids = [ARM_IDS[name] for name in ARM_JOINTS]
		msg.positions_ticks = [ticks[name] for name in ARM_JOINTS]
		self._command_pub.publish(msg)
		self._last_command_time = now

	def _publish_status(self, status: dict) -> None:
		self._status_pub.publish(
			String(data=json.dumps(status, ensure_ascii=False)))


def main(args: list[str] | None = None) -> None:
	"""노드 진입점."""
	rclpy.init(args=args)
	node = None
	try:
		node = FollowerArmNode()   # lerobot import(약 5초) 중 Ctrl+C도 여기서 받는다
		rclpy.spin(node)
	except (KeyboardInterrupt, ExternalShutdownException):
		pass   # Ctrl+C · ros2 launch 종료 (Jazzy: ExternalShutdownException)
	finally:
		# 터미널 Ctrl+C는 프로세스 그룹 전체에 가고 ros2 launch가 한 번 더 보낸다:
		# 정리하는 동안 두 번째 SIGINT가 KeyboardInterrupt로 끊지 않게 한다
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		if node is not None:
			node.destroy_node()
		rclpy.try_shutdown()


if __name__ == '__main__':
	main()
