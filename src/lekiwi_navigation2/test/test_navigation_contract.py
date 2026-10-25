"""Nav2 없이 파라미터 · launch · RViz 계약만 확인한다."""

import ast
import math
from pathlib import Path
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
LAUNCH = PACKAGE / 'launch' / 'navigation.launch.py'
PARAMS = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
SCAN_FILTER_CONFIG = (
	WORKSPACE_SRC / 'lekiwi_sensor' / 'lekiwi_scan_filter' / 'config' / 'lekiwi_scan_filter.yaml')
BASE_CONTROLLER_CONFIG = (
	WORKSPACE_SRC / 'lekiwi_motor' / 'lekiwi_base_controller' / 'config' /
	'lekiwi_base_controller.yaml')
FILTERED_SCAN = '/lekiwi/sensor/scan_filtered'

# LeKiwi 저장소 lekiwi_bringup/param/base.yaml의 차체 상한 (docs/hardware/slam_navigation.md)
REFERENCE_BASE_LIMITS = {
	'max_linear_x': 0.11,
	'max_linear_y': 0.1,
	'max_linear_speed': 0.11,
	'max_angular_z': 0.75,
}


def launch_defaults(path: Path) -> dict[str, str]:
	"""DeclareLaunchArgument 이름 → 문자열 기본값."""
	defaults = {}
	for node in ast.walk(ast.parse(path.read_text())):
		if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'DeclareLaunchArgument':
			for keyword in node.keywords:
				if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
					defaults[node.args[0].value] = keyword.value.value
	return defaults


def launch_nodes(path: Path) -> list[dict[str, str]]:
	"""Node(...) 키워드 인자를 소스 문자열로."""
	nodes = []
	for node in ast.walk(ast.parse(path.read_text())):
		if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
			nodes.append({keyword.arg: ast.unparse(keyword.value) for keyword in node.keywords})
	return nodes


def node_by_name(name: str) -> dict[str, str]:
	matches = [n for n in launch_nodes(LAUNCH) if n.get('name') == repr(name)]
	assert len(matches) == 1, name
	return matches[0]


def laser_topics() -> dict[str, str]:
	"""레이저를 받는 모든 Nav2 입력."""
	topics = {'amcl': PARAMS['amcl']['ros__parameters']['scan_topic']}
	for name in ('local_costmap', 'global_costmap'):
		topics[name] = PARAMS[name][name]['ros__parameters']['obstacle_layer']['scan']['topic']
	monitor = PARAMS['collision_monitor']['ros__parameters']
	for source in monitor['observation_sources']:
		if monitor[source]['type'] == 'scan':
			topics[f'collision_monitor.{source}'] = monitor[source]['topic']
	return topics


class NavigationContractTest(unittest.TestCase):
	def test_standard_launch_uses_saved_map_and_waits_for_localization(self):
		launch = LAUNCH.read_text()
		self.assertIn("Path.home() / 'maps' / 'lekiwi' / 'map.yaml'", launch)
		defaults = launch_defaults(LAUNCH)
		for name, value in [('use_rviz', 'false'), ('autostart', 'true'),
							('localization_autostart', 'true'), ('auto_arm_motors', 'true'),
							('motor_guard_duration', '3600'), ('use_sim_time', 'false'),
							('use_respawn', 'false'), ('operator_nodes', 'rviz,rviz2')]:
			self.assertEqual(defaults[name], value, name)
		self.assertIn('OpaqueFunction(function=validate_map_argument)', launch)
		self.assertIn('target_action=rviz_node', launch)

	def test_supervised_motor_guard_comes_from_teleop(self):
		guard = node_by_name('lekiwi_nav2_motor_guard')
		self.assertEqual(guard['package'], "'lekiwi_teleop'")
		self.assertEqual(guard['executable'], "'lekiwi_nav2_motor_guard'")
		# Pi에는 RViz가 없으므로 감독 도구는 RViz 실행과 상관없이 뜨고, 원격 RViz 노드를 지켜본다
		self.assertEqual(guard['condition'], 'IfCondition(auto_arm_motors)')
		self.assertIn("'--operator-nodes', operator_nodes", guard['arguments'])
		self.assertIn("'--yes-i-confirm-motion'", guard['arguments'])
		self.assertIn("'--max-session-duration', motor_guard_duration", guard['arguments'])

	def test_omni_controller_and_localization(self):
		amcl = PARAMS['amcl']['ros__parameters']
		self.assertEqual(amcl['robot_model_type'], 'nav2_amcl::OmniMotionModel')
		self.assertFalse(amcl['set_initial_pose'])
		controller = PARAMS['controller_server']['ros__parameters']
		mppi = controller['FollowPath']
		self.assertEqual(mppi['motion_model'], 'Omni')
		self.assertEqual(mppi['plugin'], 'nav2_mppi_controller::MPPIController')
		self.assertAlmostEqual(controller['controller_frequency'] * mppi['model_dt'], 1)
		self.assertNotIn('PreferForwardCritic', mppi['critics'])

	def test_limits_fit_reference_base_and_keep_lateral_motion(self):
		limits = PARAMS['velocity_smoother']['ros__parameters']['max_velocity']
		self.assertGreater(limits[1], 0)
		for value, name in zip(limits, ['max_linear_x', 'max_linear_y', 'max_angular_z']):
			self.assertLessEqual(value, REFERENCE_BASE_LIMITS[name])
		self.assertLessEqual(math.hypot(*limits[:2]), REFERENCE_BASE_LIMITS['max_linear_speed'])

	def test_limits_fit_workspace_base_controller(self):
		if not BASE_CONTROLLER_CONFIG.is_file():
			self.skipTest(f'{BASE_CONTROLLER_CONFIG} is not available yet')
		config = yaml.safe_load(BASE_CONTROLLER_CONFIG.read_text())
		base = config.get('lekiwi_base_controller_node', {}).get('ros__parameters', {})
		missing = [name for name in REFERENCE_BASE_LIMITS if name not in base]
		if missing:
			self.skipTest(f'base controller config has no {missing}')
		limits = PARAMS['velocity_smoother']['ros__parameters']['max_velocity']
		for value, name in zip(limits, ['max_linear_x', 'max_linear_y', 'max_angular_z']):
			self.assertLessEqual(value, base[name])
		self.assertLessEqual(math.hypot(*limits[:2]), base['max_linear_speed'])

	def test_collision_monitor_is_final_command_publisher(self):
		p = PARAMS['collision_monitor']['ros__parameters']
		self.assertEqual(p['cmd_vel_in_topic'], 'cmd_vel_smoothed')
		self.assertEqual(p['cmd_vel_out_topic'], 'cmd_vel')
		launch = LAUNCH.read_text()
		self.assertIn("'collision_monitor',", launch)
		self.assertIn("package='nav2_collision_monitor'", launch)
		self.assertNotIn("('cmd_vel_smoothed', 'cmd_vel')", launch)
		for name in ['controller_server', 'behavior_server', 'velocity_smoother']:
			self.assertIn("('cmd_vel', 'cmd_vel_nav')", node_by_name(name)['remappings'], name)
		self.assertNotIn('cmd_vel', node_by_name('collision_monitor')['remappings'])
		for name in ['controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor']:
			self.assertFalse(PARAMS[name]['ros__parameters']['enable_stamped_cmd_vel'])

	def test_every_laser_input_uses_pi_filtered_scan(self):
		topics = laser_topics()
		self.assertEqual(len(topics), 4)
		for user, topic in topics.items():
			self.assertEqual(topic, FILTERED_SCAN, user)

	def test_costmaps_keep_body_size_and_scan_loss_detection(self):
		if not SCAN_FILTER_CONFIG.is_file():
			self.skipTest(f'{SCAN_FILTER_CONFIG} is not available')
		scan_filter = yaml.safe_load(
			SCAN_FILTER_CONFIG.read_text())['lekiwi_scan_filter_node']['ros__parameters']
		# 필터 출력 토픽(상대 이름, namespace 없음)과 Nav2 입력이 같은 토픽이어야 한다.
		self.assertEqual('/' + scan_filter['output_topic'], FILTERED_SCAN)
		min_range = scan_filter['self_radius']
		self.assertFalse(scan_filter['mask_wheels'])
		self.assertAlmostEqual(PARAMS['amcl']['ros__parameters']['laser_min_range'], min_range)
		for name in ['local_costmap', 'global_costmap']:
			p = PARAMS[name][name]['ros__parameters']
			self.assertEqual(p['robot_base_frame'], 'base_footprint')
			self.assertGreaterEqual(p['robot_radius'], 0.22)
			scan = p['obstacle_layer']['scan']
			self.assertEqual(scan['sensor_frame'], 'lidar_link')
			self.assertAlmostEqual(scan['obstacle_min_range'], min_range)
			self.assertAlmostEqual(scan['raytrace_min_range'], min_range)
			self.assertLessEqual(scan['expected_update_rate'], 0.25)
			self.assertFalse(scan['inf_is_valid'])

	def test_global_costmap_waits_for_initial_pose_but_local_fails_fast(self):
		# 전역 costmap은 RViz 초기 위치(map → base_footprint)를 기다린다. 로컬은 bringup odom만 필요해 기본 60 s로 빨리 실패한다
		global_costmap = PARAMS['global_costmap']['global_costmap']['ros__parameters']
		local_costmap = PARAMS['local_costmap']['local_costmap']['ros__parameters']
		self.assertGreaterEqual(global_costmap['initial_transform_timeout'], 300.0)
		self.assertNotIn('initial_transform_timeout', local_costmap)

	def test_scan_filter_is_not_launched_on_pc(self):
		launch = LAUNCH.read_text()
		for name in ('lekiwi_sensors', 'navigation_scan', 'use_scan_filter', 'IncludeLaunchDescription'):
			self.assertNotIn(name, launch)
		packages = {n['package'] for n in launch_nodes(LAUNCH)}
		self.assertNotIn("'lekiwi_scan_filter'", packages)

	def test_rviz_shows_filtered_scan(self):
		rviz = yaml.safe_load((PACKAGE / 'rviz' / 'nav2.rviz').read_text())
		displays = {d['Name']: d for d in rviz['Visualization Manager']['Displays']}
		scan = displays['Scan']['Topic']
		self.assertEqual(scan['Value'], FILTERED_SCAN)
		self.assertEqual(scan['Reliability Policy'], 'Best Effort')

	def test_old_repository_names_are_gone(self):
		old_names = ('/scan_navigation', 'navigation_scan_filter', '/motor_ready', '/motor_power',
					 'lekiwi_node', 'lekiwi_sensors')
		for path in PACKAGE.glob('*/*'):
			if path.suffix in {'.py', '.rviz', '.yaml'} and path.parent.name != 'test':
				text = path.read_text()
				for old in old_names:
					self.assertNotIn(old, text, f'{old} in {path}')


if __name__ == '__main__':
	unittest.main()
