"""PC 도구가 쓰는 노드 · 토픽 · 서비스 이름. 기준은 src/docs/interfaces.md 다.

ROS를 import하지 않는다. distance_trial의 plan · compare가 ROS 없이 돌아야 한다.
"""

from typing import Final

# 토픽 · 서비스는 상대 이름이다. namespace가 없으면 /cmd_vel, /lekiwi/motor/power 처럼 풀린다.
CMD_VEL_TOPIC: Final[str] = 'cmd_vel'
ODOM_TOPIC: Final[str] = 'odom'
MOTOR_READY_TOPIC: Final[str] = 'lekiwi/motor/ready'
SCAN_FILTERED_TOPIC: Final[str] = 'lekiwi/sensor/scan_filtered'
MOTOR_POWER_SERVICE: Final[str] = 'lekiwi/motor/power'

# 노드 이름 (namespace 제외). 검사할 때는 qualified_node_name()으로 전체 이름을 만든다.
BASE_CONTROLLER_NODE: Final[str] = 'lekiwi_base_controller_node'
MOTOR_DRIVER_NODE: Final[str] = 'lekiwi_motor_driver_node'
NAV2_VELOCITY_NODE: Final[str] = 'collision_monitor'

# 실물 바퀴를 쓰는 베이스 컨트롤러의 motor_backend 값
DRIVER_MOTOR_BACKEND: Final[str] = 'driver'
MOCK_MOTOR_BACKEND: Final[str] = 'mock'

# 모터 드라이버 노드에서 읽는 베이스 파라미터
MOTOR_DRIVER_PARAMETERS: Final[tuple[str, ...]] = (
	'speed_tick_scale',
	'speed_tick_limit',
)


def qualified_node_name(namespace: str, node_name: str) -> str:
	"""검사하는 노드와 같은 namespace 안의 노드 전체 이름.

	Args:
		namespace: 검사하는 노드의 namespace. 기본값은 '/'.
		node_name: namespace를 뺀 노드 이름.

	Returns:
		'/<node_name>' 또는 '/<namespace>/<node_name>'.
	"""
	return f'{namespace.rstrip("/")}/{node_name}'
