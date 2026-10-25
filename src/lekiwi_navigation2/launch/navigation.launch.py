"""LeKiwi 저장 지도 주행 (Pi): map_server · Omni AMCL · Nav2 · 모터 감독 도구.

명령 경로: controller/behavior → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
→ collision_monitor → cmd_vel → (Pi) lekiwi_base_controller_node
레이저 입력은 Pi의 lekiwi_scan_filter가 발행하는 /lekiwi/sensor/scan_filtered다.
스캔 필터는 bringup이 띄우므로 여기서 띄우지 않는다.
Pi에는 화면이 없어 RViz는 PC에서 따로 연다 (use_rviz:=true는 화면이 있는 곳에서만).
감독 도구는 그 RViz 노드(operator_nodes)가 ROS 그래프에 있을 때만 토크를 켜고, 모두 사라지면 끈다.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	EmitEvent,
	GroupAction,
	LogInfo,
	OpaqueFunction,
	RegisterEventHandler,
	SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


LOCALIZATION_LIFECYCLE_NODES = [
	'map_server',
	'amcl',
]

NAVIGATION_LIFECYCLE_NODES = [
	'controller_server',
	'smoother_server',
	'planner_server',
	'behavior_server',
	'bt_navigator',
	'waypoint_follower',
	'velocity_smoother',
	'collision_monitor',
]


def validate_map_argument(context, *args, **kwargs) -> list:
	"""지도 YAML이 없으면 Nav2를 띄우기 전에 멈춘다."""
	map_value = LaunchConfiguration('map').perform(context).strip()
	if not map_value:
		raise RuntimeError('map:=/path/to/map.yaml is required for saved-map navigation')
	if not Path(map_value).expanduser().is_file():
		raise RuntimeError(
			f'Map file does not exist: {map_value}. '
			'Save a map to ~/maps/lekiwi/map.yaml or pass map:=/path/to/map.yaml.'
		)
	context.launch_configurations['map'] = str(Path(map_value).expanduser().resolve())
	return []


def make_configured_params(params_file, use_sim_time, map_file) -> ParameterFile:
	"""use_sim_time · 지도 경로를 파라미터 파일에 덮어쓴다."""
	return ParameterFile(
		RewrittenYaml(
			source_file=params_file,
			param_rewrites={
				'use_sim_time': use_sim_time,
				'yaml_filename': map_file,
			},
			convert_types=True,
		),
		allow_substs=True,
	)


def generate_launch_description() -> LaunchDescription:
	use_sim_time = LaunchConfiguration('use_sim_time')
	map_file = LaunchConfiguration('map')
	params_file = LaunchConfiguration('params_file')
	autostart = LaunchConfiguration('autostart')
	localization_autostart = LaunchConfiguration('localization_autostart')
	use_rviz = LaunchConfiguration('use_rviz')
	auto_arm_motors = LaunchConfiguration('auto_arm_motors')
	motor_guard_duration = LaunchConfiguration('motor_guard_duration')
	operator_nodes = LaunchConfiguration('operator_nodes')
	use_respawn = LaunchConfiguration('use_respawn')
	log_level = LaunchConfiguration('log_level')

	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
	use_sim_time_override = {
		'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
	}
	configured_params = make_configured_params(params_file, use_sim_time, map_file)

	use_sim_time_arg = DeclareLaunchArgument(
		'use_sim_time',
		default_value='false',
		description='Navigation2에서 시뮬레이션 시간을 사용할지 여부.',
	)
	map_arg = DeclareLaunchArgument(
		'map',
		default_value=str(Path.home() / 'maps' / 'lekiwi' / 'map.yaml'),
		description='저장된 지도 경로. 기본값은 지도 저장 위치 ~/maps/lekiwi/map.yaml입니다.',
	)
	params_file_arg = DeclareLaunchArgument(
		'params_file',
		default_value=PathJoinSubstitution([
			FindPackageShare('lekiwi_navigation2'),
			'config',
			'nav2_params.yaml',
		]),
		description='LeKiwi base mode Nav2 파라미터 파일.',
	)
	autostart_arg = DeclareLaunchArgument(
		'autostart',
		default_value='true',
		description='launch 후 Nav2 lifecycle node를 자동 활성화할지 여부.',
	)
	localization_autostart_arg = DeclareLaunchArgument(
		'localization_autostart',
		default_value='true',
		description='launch 후 map_server · AMCL을 자동 활성화할지 여부.',
	)
	use_rviz_arg = DeclareLaunchArgument(
		'use_rviz',
		default_value='false',
		description='이 컴퓨터에서 Nav2 RViz를 함께 실행할지 여부. Pi에는 화면이 없어 기본은 false (PC에서 따로 연다).',
	)
	auto_arm_motors_arg = DeclareLaunchArgument(
		'auto_arm_motors',
		default_value='true',
		description='실물 모터 감독 도구를 시작할지 여부. 실물 점검 전에는 false로 지정.',
	)
	operator_nodes_arg = DeclareLaunchArgument(
		'operator_nodes',
		default_value='rviz,rviz2',
		description='감독 도구가 지켜보는 조작 화면 노드 이름 (쉼표로 구분). '
		'하나라도 있어야 토크를 켜고, 모두 사라지면 끈다. PC에서 rviz2 -d로 열면 이름은 rviz, launch로 열면 rviz2다.',
	)
	motor_guard_duration_arg = DeclareLaunchArgument(
		'motor_guard_duration',
		default_value='3600',
		description='자동 모터 감독 도구의 최대 실행 시간(초).',
	)
	use_respawn_arg = DeclareLaunchArgument(
		'use_respawn',
		default_value='false',
		description='Nav2 node가 종료되면 자동 재시작할지 여부.',
	)
	log_level_arg = DeclareLaunchArgument(
		'log_level',
		default_value='info',
		description='Nav2 node log level.',
	)

	preflight_log = LogInfo(
		msg=[
			'LeKiwi Nav2 사전 확인: Pi bringup이 odom, /lekiwi/sensor/scan_filtered와 TF ',
			'odom -> base_footprint -> ... -> lidar_link를 발행해야 합니다. ',
			'기본 지도는 ~/maps/lekiwi/map.yaml이며, 다른 지도는 map:=/path/to/map.yaml로 지정합니다. ',
			'SLAM(lekiwi_cartographer)은 같은 ROS domain에서 함께 실행하지 않습니다. ',
			'기준 frame: map, odom, base_footprint, lidar_link. ',
			'PC에서 RViz(rviz2 -d .../nav2.rviz)를 먼저 열어야 감독 도구가 토크를 켭니다. ',
			'RViz를 닫으면 토크를 끕니다.',
		],
	)

	# ── 1. 위치 추정: 저장 지도 + Omni AMCL (map → odom) ─────────────
	localization_nodes = [
		Node(
			package='nav2_map_server',
			executable='map_server',
			name='map_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[
				configured_params,
				use_sim_time_override,
			],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		Node(
			package='nav2_amcl',
			executable='amcl',
			name='amcl',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		Node(
			package='nav2_lifecycle_manager',
			executable='lifecycle_manager',
			name='lifecycle_manager_localization',
			output='screen',
			arguments=['--ros-args', '--log-level', log_level],
			parameters=[
				configured_params,
				use_sim_time_override,
				{'autostart': ParameterValue(localization_autostart, value_type=bool)},
				{'node_names': LOCALIZATION_LIFECYCLE_NODES},
			],
		),
	]

	# ── 2. 주행: 명령은 collision_monitor만 cmd_vel로 내보낸다 ──────────
	navigation_nodes = [
		Node(
			package='nav2_controller',
			executable='controller_server',
			name='controller_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
		),
		Node(
			package='nav2_smoother',
			executable='smoother_server',
			name='smoother_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		Node(
			package='nav2_planner',
			executable='planner_server',
			name='planner_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		Node(
			package='nav2_behaviors',
			executable='behavior_server',
			name='behavior_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
		),
		Node(
			package='nav2_bt_navigator',
			executable='bt_navigator',
			name='bt_navigator',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		Node(
			package='nav2_waypoint_follower',
			executable='waypoint_follower',
			name='waypoint_follower',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
		),
		# cmd_vel_nav를 받아 cmd_vel_smoothed로 낸다.
		Node(
			package='nav2_velocity_smoother',
			executable='velocity_smoother',
			name='velocity_smoother',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params, use_sim_time_override],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings + [
				('cmd_vel', 'cmd_vel_nav'),
			],
		),
		# cmd_vel_smoothed → (충돌 감시) → cmd_vel. 토픽은 nav2_params.yaml에서 정한다.
		Node(
			package='nav2_collision_monitor',
			executable='collision_monitor',
			name='collision_monitor',
			output='screen',
			parameters=[configured_params, use_sim_time_override],
			remappings=remappings,
		),
		Node(
			package='nav2_lifecycle_manager',
			executable='lifecycle_manager',
			name='lifecycle_manager_navigation',
			output='screen',
			arguments=['--ros-args', '--log-level', log_level],
			parameters=[
				configured_params,
				use_sim_time_override,
				{'autostart': ParameterValue(autostart, value_type=bool)},
				{'node_names': NAVIGATION_LIFECYCLE_NODES},
			],
		),
	]

	# ── 3. RViz와 모터 감독 도구 ───────────────────────────────────
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=[
			'-d',
			PathJoinSubstitution([
				FindPackageShare('lekiwi_navigation2'),
				'rviz',
				'nav2.rviz',
			]),
		],
		condition=IfCondition(use_rviz),
	)
	# 감독 도구가 lekiwi/motor/power로 토크를 켜고, 상태가 나빠지거나 조작 화면이 닫히거나
	# 시간이 끝나면 끈다.
	motor_guard_node = Node(
		package='lekiwi_teleop',
		executable='lekiwi_nav2_motor_guard',
		name='lekiwi_nav2_motor_guard',
		output='screen',
		arguments=[
			'--yes-i-confirm-motion',
			'--startup-timeout', '60',
			'--max-session-duration', motor_guard_duration,
			'--operator-nodes', operator_nodes,
		],
		sigterm_timeout='10',
		condition=IfCondition(auto_arm_motors),
	)
	# 이 컴퓨터에서 연 RViz를 닫으면 launch 전체를 끝낸다 (원격 RViz는 감독 도구가 지켜본다).
	stop_with_rviz = RegisterEventHandler(
		OnProcessExit(
			target_action=rviz_node,
			on_exit=[EmitEvent(event=Shutdown(reason='Nav2 RViz closed'))],
		),
		condition=IfCondition(AndSubstitution(use_rviz, auto_arm_motors)),
	)

	return LaunchDescription([
		SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
		use_sim_time_arg,
		map_arg,
		params_file_arg,
		autostart_arg,
		localization_autostart_arg,
		use_rviz_arg,
		auto_arm_motors_arg,
		motor_guard_duration_arg,
		operator_nodes_arg,
		use_respawn_arg,
		log_level_arg,
		preflight_log,
		OpaqueFunction(function=validate_map_argument),
		GroupAction(actions=[
			SetParameter(
				name='use_sim_time',
				value=ParameterValue(use_sim_time, value_type=bool),
			),
			*localization_nodes,
			*navigation_nodes,
		]),
		stop_with_rviz,
		rviz_node,
		motor_guard_node,
	])
