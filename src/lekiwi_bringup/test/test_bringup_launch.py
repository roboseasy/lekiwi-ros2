"""bringup.launch.py의 노드 구성과 장비 프로파일 선택을 시험한다.

다른 패키지가 설치되지 않은 PC에서도 돌도록 패키지 경로 찾기를 가짜로 바꾼다.
"""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent   # 워크스페이스 src


def load_launch(path, name='bringup'):
	spec = importlib.util.spec_from_file_location(name, path)
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


def load_bringup():
	return load_launch(PACKAGE / 'launch' / 'bringup.launch.py')


@pytest.fixture
def bringup(monkeypatch, tmp_path):
	module = load_bringup()
	installed = {
		'lekiwi_bringup', 'lekiwi_description', 'lekiwi_imu', 'lekiwi_lidar', 'lekiwi_scan_filter',
		'lekiwi_motor_driver', 'lekiwi_base_controller'}

	def fake_share(package):
		if package not in installed:
			raise PackageNotFoundError(package)
		if package == 'lekiwi_bringup':
			return str(PACKAGE)
		return f'/share/{package}'

	monkeypatch.setattr(module, 'share', fake_share)
	# Node 대신 받은 인자를 그대로 돌려줘 이름 · 파라미터를 비교한다.
	monkeypatch.setattr(module, 'make_node', lambda **kwargs: kwargs)
	monkeypatch.setattr(module, 'USER_PROFILE', str(tmp_path / 'no_user_profile.yaml'))
	monkeypatch.setattr(module, 'ARM_PROFILE', str(tmp_path / 'no_arm_profile.yaml'))
	module.installed = installed
	return module


def run(module, **arguments):
	defaults = {
		'robot_profile': '', 'use_imu': 'true', 'use_lidar': 'true', 'use_camera': 'false',
		'use_motors': 'true', 'use_base_controller': 'true', 'use_arm': 'true',
		'lerobot_python': '/no/lerobot/python', 'robot_id': '', 'joint_state_rate': '10',
		'use_zmq_host': 'true', 'zmq_cmd_port': '5555', 'zmq_obs_port': '5556'}
	defaults.update(arguments)
	context = LaunchContext()
	for name, value in defaults.items():
		context.launch_configurations[name] = value
	return module.launch_setup(context)


def nodes(actions):
	return {action['name']: action for action in actions if isinstance(action, dict)}


def parameters(node):
	return node['parameters']


def log_messages(actions):
	return [
		''.join(getattr(part, 'text', '') for part in action.msg)
		for action in actions if isinstance(action, LogInfo)]


def install_follower(bringup, tmp_path):
	"""lekiwi_lerobot 설치(설정 파일)와 lerobot 가상환경 python을 가짜로 만든다."""
	share_dir = tmp_path / 'lekiwi_lerobot'
	(share_dir / 'config').mkdir(parents=True)
	(share_dir / 'config' / 'follower_arm.yaml').write_text('{}\n')
	python = tmp_path / 'python'
	python.write_text('')
	bringup.installed.add('lekiwi_lerobot')
	original = bringup.share
	bringup.share = lambda package: str(share_dir) if package == 'lekiwi_lerobot' else original(package)
	return share_dir, python


def test_default_starts_model_sensors_and_motors(bringup):
	started = nodes(run(bringup))
	assert set(started) == {
		'robot_state_publisher', 'joint_state_publisher', 'lekiwi_imu_node', 'lekiwi_lidar_node',
		'lekiwi_scan_filter_node', 'lekiwi_motor_driver_node', 'lekiwi_base_controller_node'}


def test_driver_writes_only_with_base_controller(bringup):
	driver = nodes(run(bringup))['lekiwi_motor_driver_node']
	assert {'wheel_write_enabled': True} in parameters(driver)

	started = nodes(run(bringup, use_base_controller='false'))
	assert 'lekiwi_base_controller_node' not in started
	assert {'wheel_write_enabled': False} in parameters(started['lekiwi_motor_driver_node'])


def test_base_frame_is_loaded_after_controller_config_and_before_profile(bringup):
	controller = nodes(run(bringup))['lekiwi_base_controller_node']
	files = [str(item) for item in parameters(controller)]
	assert files[0].endswith('lekiwi_base_controller/config/lekiwi_base_controller.yaml')
	assert files[1].endswith('lekiwi_description/config/base_frame.yaml')
	assert files[2].endswith('lekiwi_bringup/config/robot_profile.yaml')


def test_joint_states_default_to_ten_hertz_and_can_be_raised(bringup):
	# 50 Hz였을 때 Pi에서 joint_state_publisher 하나가 CPU 33%를 쓰고 /tf가 약 67 Hz였다
	publisher = nodes(run(bringup))['joint_state_publisher']
	assert parameters(publisher)[0]['rate'] == 10
	assert parameters(publisher)[0]['source_list'] == ['lekiwi/motor/wheel_joint_states']
	publisher = nodes(run(bringup, joint_state_rate='30'))['joint_state_publisher']
	assert parameters(publisher)[0]['rate'] == 30
	with pytest.raises(ValueError):
		run(bringup, joint_state_rate='0')


def test_sensors_and_motors_can_be_turned_off(bringup):
	started = nodes(run(bringup, use_imu='false', use_lidar='false', use_motors='false'))
	assert set(started) == {'robot_state_publisher', 'joint_state_publisher'}


def test_camera_skipped_when_package_missing(bringup):
	actions = run(bringup, use_camera='true')
	assert 'front_camera_node' not in nodes(actions)
	assert any('lekiwi_camera' in message for message in log_messages(actions))


def test_camera_started_when_package_present(bringup):
	bringup.installed.add('lekiwi_camera')
	started = nodes(run(bringup, use_camera='true'))
	assert {'front_camera_node', 'wrist_camera_node'} <= set(started)


def test_driver_arm_write_follows_use_arm(bringup):
	driver = nodes(run(bringup))['lekiwi_motor_driver_node']
	assert {'arm_write_enabled': True} in parameters(driver)

	actions = run(bringup, use_arm='false')
	assert {'arm_write_enabled': False} in parameters(nodes(actions)['lekiwi_motor_driver_node'])
	assert 'lekiwi_follower_arm_node' not in nodes(actions)
	assert not any('follower' in message for message in log_messages(actions))


def test_follower_skipped_when_package_missing(bringup):
	actions = run(bringup)
	assert 'lekiwi_follower_arm_node' not in nodes(actions)
	assert any('lekiwi_lerobot' in message for message in log_messages(actions))


def test_follower_skipped_when_lerobot_python_missing(bringup, tmp_path):
	install_follower(bringup, tmp_path)
	actions = run(bringup, lerobot_python=str(tmp_path / 'missing_python'))
	assert 'lekiwi_follower_arm_node' not in nodes(actions)
	assert any('missing_python' in message for message in log_messages(actions))


def test_follower_gets_config_then_profile_and_lerobot_python(bringup, tmp_path):
	share_dir, python = install_follower(bringup, tmp_path)
	follower = nodes(run(bringup, lerobot_python=str(python)))['lekiwi_follower_arm_node']
	assert (follower['package'], follower['executable']) == ('lekiwi_lerobot', 'lekiwi_follower_arm_node')
	assert follower['prefix'] == str(python)
	# 다른 노드와 같은 순서. robot_id를 주지 않으면 넘기지 않는다 (프로파일 값이 이긴다)
	assert parameters(follower) == [
		str(share_dir / 'config' / 'follower_arm.yaml'), str(PACKAGE / 'config' / 'robot_profile.yaml')]
	follower = nodes(run(bringup, lerobot_python=str(python), robot_id='04'))['lekiwi_follower_arm_node']
	[robot_id] = parameters(follower)[2:]
	assert (robot_id['robot_id'].value, robot_id['robot_id'].value_type) == ('04', str)
	# 팔로워는 드라이버가 있을 때만 켠다
	assert 'lekiwi_follower_arm_node' not in nodes(run(bringup, lerobot_python=str(python), use_motors='false'))


def test_follower_skips_a_missing_arm_profile_with_a_log_line(bringup, tmp_path):
	share_dir, python = install_follower(bringup, tmp_path)
	actions = run(bringup, lerobot_python=str(python))
	assert parameters(nodes(actions)['lekiwi_follower_arm_node']) == [
		str(share_dir / 'config' / 'follower_arm.yaml'), str(PACKAGE / 'config' / 'robot_profile.yaml')]
	assert f'arm profile: {bringup.ARM_PROFILE} not found; skipping it' in log_messages(actions)


def test_follower_loads_the_arm_profile_after_the_robot_profile_and_before_robot_id(bringup, tmp_path):
	share_dir, python = install_follower(bringup, tmp_path)
	arm_profile = tmp_path / 'arm_profile.yaml'
	arm_profile.write_text('lekiwi_follower_arm_node:\n  ros__parameters:\n    robot_id: lekiwi01\n')
	bringup.ARM_PROFILE = str(arm_profile)
	actions = run(bringup, lerobot_python=str(python))
	assert parameters(nodes(actions)['lekiwi_follower_arm_node']) == [
		str(share_dir / 'config' / 'follower_arm.yaml'), str(PACKAGE / 'config' / 'robot_profile.yaml'),
		str(arm_profile)]
	assert f'arm profile: {arm_profile}' in log_messages(actions)
	# 다른 노드에는 넣지 않는다
	assert all(
		str(arm_profile) not in parameters(node) for name, node in nodes(actions).items()
		if name != 'lekiwi_follower_arm_node')
	follower = nodes(run(bringup, lerobot_python=str(python), robot_id='04'))['lekiwi_follower_arm_node']
	assert parameters(follower)[2] == str(arm_profile)
	[robot_id] = parameters(follower)[3:]
	assert robot_id['robot_id'].value == '04'


def follower_section(**values):
	return yaml.safe_dump({'lekiwi_follower_arm_node': {'ros__parameters': values}})


def test_arm_profile_wins_over_robot_profile_and_robot_id_argument_wins_over_both(bringup, tmp_path):
	"""launch가 넣는 순서대로 rclpy에 params 파일을 주면 뒤의 값이 이긴다 (실제 rclpy로 확인)."""
	share_dir, python = install_follower(bringup, tmp_path)
	(share_dir / 'config' / 'follower_arm.yaml').write_text(follower_section(
		robot_id='', rest_pose_ticks=[], rate_hz=30.0))
	robot_profile = tmp_path / 'robot_profile.yaml'
	robot_profile.write_text(follower_section(robot_id='hand', rest_pose_ticks=[1, 2, 3, 4, 5, 6], max_step_deg=8.0))
	arm_profile = tmp_path / 'arm_profile.yaml'
	arm_profile.write_text(follower_section(robot_id='lekiwi01', rest_pose_ticks=[2054, 891, 3110, 2787, 2292, 2014]))
	bringup.ARM_PROFILE = str(arm_profile)

	def effective(**arguments):
		follower = nodes(run(bringup, lerobot_python=str(python), robot_profile=str(robot_profile), **arguments))[
			'lekiwi_follower_arm_node']
		files = []
		for index, entry in enumerate(parameters(follower)):
			if isinstance(entry, dict):   # launch_ros는 dict 파라미터도 이 자리의 params 파일로 넘긴다
				entry = tmp_path / f'dict_{index}.yaml'
				entry.write_text(follower_section(robot_id=parameters(follower)[index]['robot_id'].value))
			files += ['--params-file', str(entry)]
		return read_follower_parameters(files)

	assert effective() == {
		'robot_id': 'lekiwi01', 'rest_pose_ticks': [2054, 891, 3110, 2787, 2292, 2014],
		'max_step_deg': 8.0, 'rate_hz': 30.0}
	assert effective(robot_id='04')['robot_id'] == '04'


def read_follower_parameters(ros_arguments):
	"""lekiwi_follower_arm_node 이름의 rclpy 노드를 그 인자로 띄워 몇 파라미터 값을 읽는다 (통신하지 않는다)."""
	script = (
		'import json, sys, rclpy\n'
		'from rcl_interfaces.msg import ParameterDescriptor\n'
		'rclpy.init(args=sys.argv)\n'
		'node = rclpy.create_node("lekiwi_follower_arm_node")\n'
		'any_type = ParameterDescriptor(dynamic_typing=True)\n'
		'names = ("robot_id", "rest_pose_ticks", "max_step_deg", "rate_hz")\n'
		'values = {name: node.declare_parameter(name, None, any_type).value for name in names}\n'
		'print(json.dumps({name: list(v) if hasattr(v, "tolist") or isinstance(v, (list, tuple)) else v\n'
		'    for name, v in values.items()}))\n'
		'rclpy.shutdown()\n')
	env = os.environ | {
		'ROS_DOMAIN_ID': os.environ.get('LEKIWI_TEST_ROS_DOMAIN_ID', str(60 + os.getpid() % 20)),
		'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
	done = subprocess.run(
		[sys.executable, '-c', script, '--ros-args', *ros_arguments],
		env=env, capture_output=True, text=True, timeout=30, check=True)
	return json.loads(done.stdout.strip().splitlines()[-1])


def test_real_follower_node_uses_lerobot_python_and_does_not_shut_down_bringup(
	bringup, tmp_path, monkeypatch,
):
	share_dir, python = install_follower(bringup, tmp_path)
	monkeypatch.setattr(bringup, 'make_node', Node)   # 이 시험만 실제 launch_ros Node를 만든다
	[follower] = [
		action for action in run(bringup, lerobot_python=str(python))
		if isinstance(action, Node) and action.node_package == 'lekiwi_lerobot']
	assert perform_substitutions(LaunchContext(), follower.process_description.prefix) == str(python)
	assert follower._ExecuteLocal__on_exit is None   # 팔로워가 끝나도 Shutdown 등을 걸지 않는다

	# lekiwi_lerobot의 단독 실행 launch와 같은 노드(패키지 · 실행 파일 · 이름)를 같은 prefix로 켠다
	standalone_launch = load_launch(
		SOURCE / 'lekiwi_lerobot' / 'launch' / 'follower_arm.launch.py', 'follower_arm_launch')
	monkeypatch.setattr(standalone_launch, 'get_package_share_directory', lambda package: str(share_dir))
	context = LaunchContext()
	context.launch_configurations['lerobot_python'] = str(python)
	[standalone] = expand_nodes(standalone_launch.generate_launch_description(), context)
	assert (standalone.node_package, standalone.node_executable, standalone._Node__node_name) == (
		follower.node_package, follower.node_executable, follower._Node__node_name)
	assert perform_substitutions(context, standalone.process_description.prefix) == str(python)
	assert standalone._ExecuteLocal__on_exit is None


def expand_nodes(description, context):
	"""launch를 실행하지 않고 인자 기본값 · OpaqueFunction만 펼쳐 Node를 모은다."""
	found = []
	pending = list(description.entities)
	while pending:
		entity = pending.pop(0)
		if isinstance(entity, Node):
			found.append(entity)
		elif isinstance(entity, (DeclareLaunchArgument, OpaqueFunction)):
			pending[0:0] = entity.execute(context) or []
	return found


def test_zmq_host_skipped_with_a_log_line_when_package_missing(bringup):
	actions = run(bringup)
	assert 'lekiwi_zmq_host_node' not in nodes(actions)
	assert any('lekiwi_zmq_host' in message for message in log_messages(actions))


def test_zmq_host_runs_on_system_python_with_port_arguments_last(bringup):
	bringup.installed.add('lekiwi_zmq_host')
	host = nodes(run(bringup))['lekiwi_zmq_host_node']
	assert (host['package'], host['executable']) == ('lekiwi_zmq_host', 'lekiwi_zmq_host_node')
	assert 'prefix' not in host   # lerobot 가상환경을 쓰지 않는다
	assert parameters(host) == [
		'/share/lekiwi_zmq_host/config/lekiwi_zmq_host.yaml', str(PACKAGE / 'config' / 'robot_profile.yaml'),
		{'port_zmq_cmd': 5555, 'port_zmq_observations': 5556}]
	host = nodes(run(bringup, zmq_cmd_port='6555', zmq_obs_port='6556'))['lekiwi_zmq_host_node']
	assert parameters(host)[-1] == {'port_zmq_cmd': 6555, 'port_zmq_observations': 6556}


def test_zmq_host_can_be_turned_off_and_needs_the_motors(bringup):
	bringup.installed.add('lekiwi_zmq_host')
	for arguments in ({'use_zmq_host': 'false'}, {'use_motors': 'false'}):
		actions = run(bringup, **arguments)
		assert 'lekiwi_zmq_host_node' not in nodes(actions)
		assert not any('zmq' in message.lower() for message in log_messages(actions))


def test_profile_resolution_order(bringup, tmp_path):
	default_profile = str(PACKAGE / 'config' / 'robot_profile.yaml')
	assert bringup.resolve_profile(context_with(robot_profile='')) == default_profile

	user = tmp_path / 'robot_profile.yaml'
	user.write_text('{}\n')
	bringup.USER_PROFILE = str(user)
	assert bringup.resolve_profile(context_with(robot_profile='')) == str(user)

	explicit = tmp_path / 'other.yaml'
	explicit.write_text('{}\n')
	assert bringup.resolve_profile(context_with(robot_profile=str(explicit))) == str(explicit)

	with pytest.raises(FileNotFoundError):
		bringup.resolve_profile(context_with(robot_profile=str(tmp_path / 'missing.yaml')))


def context_with(**arguments):
	context = LaunchContext()
	for name, value in arguments.items():
		context.launch_configurations[name] = value
	return context


def test_default_profile_only_names_launched_nodes():
	profile = yaml.safe_load((PACKAGE / 'config' / 'robot_profile.yaml').read_text())
	assert set(profile) <= {
		'lekiwi_motor_driver_node', 'lekiwi_base_controller_node', 'lekiwi_lidar_node', 'lekiwi_imu_node',
		'lekiwi_scan_filter_node', 'front_camera_node', 'wrist_camera_node', 'lekiwi_follower_arm_node',
		'lekiwi_zmq_host_node'}
	for section in profile.values():
		assert set(section) == {'ros__parameters'}
	controller = profile['lekiwi_base_controller_node']['ros__parameters']
	driver = profile['lekiwi_motor_driver_node']['ros__parameters']
	# 베이스 컨트롤러가 쓰는 바퀴는 드라이버가 쓰기를 허용한 바퀴와 같아야 한다.
	assert set(controller['wheel_ids']) == set(driver['wheel_ids'])
	assert len(controller['wheel_directions']) == 3
	assert all(direction in (-1.0, 1.0) for direction in controller['wheel_directions'])


def test_launch_description_declares_arguments():
	module = load_bringup()
	description = module.generate_launch_description()
	names = {entity.name for entity in description.entities if hasattr(entity, 'default_value')}
	assert names == {
		'robot_profile', 'use_imu', 'use_lidar', 'use_camera', 'use_motors', 'use_base_controller',
		'use_arm', 'lerobot_python', 'robot_id', 'joint_state_rate', 'use_zmq_host', 'zmq_cmd_port',
		'zmq_obs_port'}
	assert os.path.basename(module.USER_PROFILE) == 'robot_profile.yaml'
	defaults = {
		entity.name: entity.default_value for entity in description.entities
		if hasattr(entity, 'default_value')}
	assert ''.join(part.text for part in defaults['use_arm']) == 'true'
	assert ''.join(part.text for part in defaults['use_zmq_host']) == 'true'
	assert ''.join(part.text for part in defaults['zmq_cmd_port']) == '5555'
	assert ''.join(part.text for part in defaults['zmq_obs_port']) == '5556'
	# 계정 이름을 박지 않고 홈 기준으로 찾는다 (lekiwi04: /home/roboseasy/lerobot060_venv/bin/python)
	assert module.LEROBOT_PYTHON == os.path.join(
		os.path.expanduser('~'), 'lerobot060_venv', 'bin', 'python')


def test_base_wheel_limit_matches_driver_speed_tick_limit():
	"""베이스 컨트롤러 max_wheel_speed [rad/s] × 드라이버 speed_tick_scale = 드라이버 speed_tick_limit (짝으로 맞춘다)."""
	controller = yaml.safe_load(
		(SOURCE / 'lekiwi_motor' / 'lekiwi_base_controller' / 'config' / 'lekiwi_base_controller.yaml').read_text())
	driver = yaml.safe_load(
		(SOURCE / 'lekiwi_motor' / 'lekiwi_motor_driver' / 'config' / 'lekiwi_motor_driver.yaml').read_text())
	controller = controller['lekiwi_base_controller_node']['ros__parameters']
	driver = driver['lekiwi_motor_driver_node']['ros__parameters']
	assert driver['speed_tick_limit'] == 3000
	assert round(controller['max_wheel_speed'] * driver['speed_tick_scale']) == driver['speed_tick_limit']
