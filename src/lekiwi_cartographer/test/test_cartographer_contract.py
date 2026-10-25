"""cartographer_ros 없이 launch · Lua · RViz 설정 계약만 확인한다."""

import ast
import os
from pathlib import Path
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'cartographer.launch.py'


def launch_defaults(path: Path) -> dict[str, str]:
	"""DeclareLaunchArgument 이름 → 문자열 기본값."""
	defaults = {}
	for node in ast.walk(ast.parse(path.read_text())):
		if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'DeclareLaunchArgument':
			for keyword in node.keywords:
				if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
					defaults[node.args[0].value] = keyword.value.value
	return defaults


def launch_nodes(path: Path) -> list[dict[str, object]]:
	"""Node(...)의 상수 키워드 인자 (package, executable, remappings 등)."""
	nodes = []
	for node in ast.walk(ast.parse(path.read_text())):
		if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
			fields = {}
			for keyword in node.keywords:
				try:
					fields[keyword.arg] = ast.literal_eval(keyword.value)
				except ValueError:
					fields[keyword.arg] = ast.unparse(keyword.value)
			nodes.append(fields)
	return nodes


class CartographerContractTest(unittest.TestCase):
	def test_standard_launch_selects_imu_profile_and_pi_topics(self):
		defaults = launch_defaults(LAUNCH)
		self.assertEqual(defaults['configuration_basename'], 'lekiwi_2d_imu.lua')
		self.assertEqual(defaults['scan_topic'], 'lekiwi/sensor/scan_filtered')
		self.assertEqual(defaults['imu_topic'], 'lekiwi/sensor/offset_imu')
		self.assertEqual(defaults['resolution'], '0.05')
		self.assertEqual(defaults['publish_period_sec'], '1.0')
		self.assertEqual(defaults['use_rviz'], 'false')    # Pi에는 화면이 없다
		self.assertEqual(defaults['use_sim_time'], 'false')

	def test_cartographer_remaps_scan_and_imu_but_keeps_odom(self):
		cartographer = [n for n in launch_nodes(LAUNCH) if n.get('executable') == 'cartographer_node']
		self.assertEqual(len(cartographer), 1)
		remappings = cartographer[0]['remappings']
		self.assertIn("('scan', scan_topic)", remappings)
		self.assertIn("('imu', imu_topic)", remappings)
		self.assertNotIn('odom', remappings)

	def test_scan_filter_is_left_to_bringup(self):
		launch = LAUNCH.read_text()
		for name in ('lekiwi_sensors', 'navigation_scan', 'use_scan_filter', "'lekiwi_scan_filter'"):
			self.assertNotIn(name, launch)
		# include는 occupancy grid 하나뿐이다.
		self.assertEqual(launch.count('IncludeLaunchDescription('), 1)
		self.assertIn("'occupancy_grid.launch.py'", launch)
		packages = {n.get('package') for n in launch_nodes(LAUNCH)}
		self.assertEqual(packages, {'cartographer_ros', 'rviz2'})
		grid = launch_nodes(PACKAGE / 'launch' / 'occupancy_grid.launch.py')
		self.assertEqual([n['executable'] for n in grid], ['cartographer_occupancy_grid_node'])
		self.assertEqual(launch_defaults(PACKAGE / 'launch' / 'occupancy_grid.launch.py')['resolution'], '0.05')

	def test_odom_has_single_owner_and_imu_is_optional(self):
		for name, frame, imu in [('lekiwi_2d.lua', 'base_footprint', 'false'),
								 ('lekiwi_2d_imu.lua', 'imu_link', 'true')]:
			lua = (PACKAGE / 'config' / name).read_text()
			self.assertIn('published_frame = "odom"', lua)
			self.assertIn('provide_odom_frame = false', lua)
			self.assertIn('use_odometry = true', lua)
			self.assertIn(f'tracking_frame = "{frame}"', lua)
			self.assertIn(f'use_imu_data = {imu}', lua)
			for setting in ['max_range = 9.0', 'occupied_space_weight = 20.0',
							'num_range_data = 45', 'num_threads = 2']:
				self.assertIn(setting, lua)

	def test_rviz_shows_filtered_scan_on_map(self):
		rviz = yaml.safe_load((PACKAGE / 'rviz' / 'slam.rviz').read_text())
		manager = rviz['Visualization Manager']
		self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
		displays = {d['Class']: d for d in manager['Displays']}
		scan = displays['rviz_default_plugins/LaserScan']['Topic']
		self.assertEqual(scan['Value'], '/lekiwi/sensor/scan_filtered')
		self.assertEqual(scan['Reliability Policy'], 'Best Effort')
		self.assertEqual(displays['rviz_default_plugins/Map']['Topic']['Value'], '/map')

	def test_old_repository_names_are_gone(self):
		for path in PACKAGE.glob('*/*'):
			if path.suffix in {'.py', '.lua', '.rviz', '.sh', '.yaml'} and path.parent.name != 'test':
				text = path.read_text()
				for old in ('/scan_navigation', 'imu/data_raw', 'navigation_scan_filter', 'lekiwi_node'):
					self.assertNotIn(old, text, f'{old} in {path}')

	def test_save_map_script_is_installed_as_executable(self):
		script = PACKAGE / 'scripts' / 'save_map.sh'
		self.assertTrue(os.access(script, os.X_OK))
		cmake = (PACKAGE / 'CMakeLists.txt').read_text()
		self.assertIn('install(PROGRAMS scripts/save_map.sh', cmake)
		self.assertIn('DESTINATION lib/${PROJECT_NAME}', cmake)


if __name__ == '__main__':
	unittest.main()
