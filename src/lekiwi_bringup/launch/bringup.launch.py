"""LeKiwi Pi 브링업: 로봇 모델 · 센서 · 모터를 한 번에 켠다.

섹션
  1. 로봇 모델   robot_state_publisher, joint_state_publisher
  2. 센서        lekiwi_imu, lekiwi_lidar, lekiwi_scan_filter, lekiwi_camera(선택)
  3. 모터        lekiwi_motor_driver(버스를 여는 유일한 노드), lekiwi_base_controller,
                 lekiwi_lerobot follower_arm(use_arm, 설치돼 있고 lerobot_python이 있을 때만)

노드마다 파라미터를 [패키지 기본 설정 → (베이스 컨트롤러만) base_frame.yaml → 장비 프로파일
→ (팔로워만, 있으면) ~/lekiwi_ws/arm_profile.yaml → (팔로워만, 주면) robot_id 인자] 순서로 넣는다. 뒤에 넣은 값이 이긴다.
장비 프로파일은 robot_profile 인자, 없으면 ~/lekiwi_ws/robot_profile.yaml, 그것도 없으면 이 패키지의 config/robot_profile.yaml을 쓴다.
arm_profile.yaml은 웹 게이트웨이(보정 탭)가 통째로 쓰는 팔로워 파일이다 (robot_id · rest_pose_ticks). 사람이 쓴 장비 프로파일은 고치지 않는다.

바퀴 토크는 켜지 않는다. 움직이려면 lekiwi/motor/power 서비스로 켠다 (lekiwi_teleop 참고).
팔 토크도 바꾸지 않는다. 팔은 lekiwi/motor/arm_torque로 켠 뒤에만 움직인다.
"""

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 장비마다 다른 값을 두는 파일 (워크스페이스 src 밖이라 git에 섞이지 않는다)
USER_PROFILE = os.path.join(os.path.expanduser('~'), 'lekiwi_ws', 'robot_profile.yaml')
# 게이트웨이가 쓰는 팔로워 팔 파일 (lekiwi_follower_arm_node 섹션만). 없으면 건너뛴다
ARM_PROFILE = os.path.join(os.path.expanduser('~'), 'lekiwi_ws', 'arm_profile.yaml')
# 팔로워 팔 노드를 돌리는 lerobot 0.6.0 가상환경의 python (ROS 설정은 launch가 이미 source한 것을 쓴다)
LEROBOT_PYTHON = os.path.join(os.path.expanduser('~'), 'lerobot060_venv', 'bin', 'python')


def is_true(context, name):
	return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def share(package):
	return get_package_share_directory(package)


def make_node(**kwargs):
	return Node(**kwargs)


def resolve_profile(context):
	"""robot_profile 인자 → ~/lekiwi_ws/robot_profile.yaml → 패키지 기본값 순서로 고른다."""
	requested = os.path.expanduser(LaunchConfiguration('robot_profile').perform(context))
	if requested:
		if not os.path.isfile(requested):
			raise FileNotFoundError(f'robot_profile not found: {requested}')
		return requested
	if os.path.isfile(USER_PROFILE):
		return USER_PROFILE
	return os.path.join(share('lekiwi_bringup'), 'config', 'robot_profile.yaml')


def launch_setup(context):
	profile = resolve_profile(context)
	actions = [LogInfo(msg=f'robot profile: {profile}')]

	# ── 1. 로봇 모델 ───────────────────────────────────────────────
	urdf = os.path.join(share('lekiwi_description'), 'urdf', 'lekiwi.urdf.xacro')
	robot_description = ParameterValue(
		Command([FindExecutable(name='xacro'), ' ', urdf, ' use_hardware_sensors:=true']),
		value_type=str)
	actions.append(make_node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': robot_description}],
	))
	# 바퀴 관절은 베이스 컨트롤러 값을 쓰고, 팔 관절은 0으로 채운다 (팔 각도 보정 전).
	# 이 주기로 robot_state_publisher가 /tf를 내고, TF를 듣는 모든 노드(Nav2 · Cartographer)가 그만큼 받는다.
	# 바퀴 · 팔 관절은 화면 표시용이라 10 Hz면 된다. 주행용 odom → base_footprint는 베이스 컨트롤러가 따로 낸다.
	joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))
	if joint_state_rate <= 0:
		raise ValueError(f'joint_state_rate must be positive, got {joint_state_rate}')
	actions.append(make_node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		output='screen',
		parameters=[{
			'source_list': ['lekiwi/motor/wheel_joint_states'],
			'rate': joint_state_rate,
			'zeros.wrist_roll': -1.5707963267948966,
		}],
	))

	# ── 2. 센서 ────────────────────────────────────────────────────
	if is_true(context, 'use_imu'):
		actions.append(make_node(
			package='lekiwi_imu',
			executable='lekiwi_imu_node',
			name='lekiwi_imu_node',
			output='screen',
			parameters=[os.path.join(share('lekiwi_imu'), 'config', 'lekiwi_imu.yaml'), profile],
		))
	if is_true(context, 'use_lidar'):
		actions.append(make_node(
			package='lekiwi_lidar',
			executable='lekiwi_lidar_node',
			name='lekiwi_lidar_node',
			output='screen',
			parameters=[os.path.join(share('lekiwi_lidar'), 'config', 'lekiwi_lidar.yaml'), profile],
		))
		actions.append(make_node(
			package='lekiwi_scan_filter',
			executable='lekiwi_scan_filter_node',
			name='lekiwi_scan_filter_node',
			output='screen',
			parameters=[
				os.path.join(share('lekiwi_scan_filter'), 'config', 'lekiwi_scan_filter.yaml'), profile],
		))
	if is_true(context, 'use_camera'):
		try:
			camera_config = os.path.join(share('lekiwi_camera'), 'config', 'lekiwi_camera.yaml')
		except PackageNotFoundError:
			actions.append(LogInfo(msg='use_camera:=true but lekiwi_camera is not built; skipping cameras'))
		else:
			for name in ('front_camera_node', 'wrist_camera_node'):
				actions.append(make_node(
					package='lekiwi_camera',
					executable='lekiwi_camera_node',
					name=name,
					output='screen',
					parameters=[camera_config, profile],
				))

	# ── 3. 모터 ────────────────────────────────────────────────────
	if is_true(context, 'use_motors'):
		use_base_controller = is_true(context, 'use_base_controller')
		use_arm = is_true(context, 'use_arm')
		# 같은 버스를 쓰는 다른 프로그램(LeRobot 호스트, 저장소 lekiwi_node)과 동시에 실행하지 않는다.
		actions.append(make_node(
			package='lekiwi_motor_driver',
			executable='lekiwi_motor_driver_node',
			name='lekiwi_motor_driver_node',
			output='screen',
			parameters=[
				os.path.join(share('lekiwi_motor_driver'), 'config', 'lekiwi_motor_driver.yaml'),
				profile,
				{'wheel_write_enabled': use_base_controller},
				{'arm_write_enabled': use_arm},
			],
		))
		if use_base_controller:
			actions.append(make_node(
				package='lekiwi_base_controller',
				executable='lekiwi_base_controller_node',
				name='lekiwi_base_controller_node',
				output='screen',
				parameters=[
					os.path.join(share('lekiwi_base_controller'), 'config', 'lekiwi_base_controller.yaml'),
					os.path.join(share('lekiwi_description'), 'config', 'base_frame.yaml'),
					profile,
				],
			))
		if use_arm:
			actions.extend(follower_arm(context, profile))
	return actions


def follower_arm(context, profile):
	"""lekiwi_lerobot이 설치돼 있고 lerobot_python이 있을 때만 팔로워 팔 노드를 켠다. 없으면 건너뛴다 (로그 한 줄).

	lekiwi_lerobot/launch/follower_arm.launch.py와 같은 노드를 같은 방법(lerobot_python을 prefix로)으로 켠다.
	그 launch는 파라미터 파일을 하나만 받고 robot_id를 늘 덮어써서, 다른 노드처럼 장비 프로파일을 얹으려고 여기서 직접 만든다.
	팔로워가 죽어도 bringup은 계속 돈다 (on_exit 종료를 걸지 않는다). 드라이버는 마지막 목표를 유지한다.
	"""
	lerobot_python = os.path.expanduser(LaunchConfiguration('lerobot_python').perform(context))
	try:
		config = os.path.join(share('lekiwi_lerobot'), 'config', 'follower_arm.yaml')
	except PackageNotFoundError:
		return [LogInfo(msg='use_arm:=true but lekiwi_lerobot is not built; skipping the follower arm')]
	for path in (config, lerobot_python):
		if not os.path.isfile(path):
			return [LogInfo(msg=f'use_arm:=true but {path} does not exist; skipping the follower arm')]
	parameters = [config, profile]
	if os.path.isfile(ARM_PROFILE):
		parameters.append(ARM_PROFILE)
		arm_log = LogInfo(msg=f'arm profile: {ARM_PROFILE}')
	else:
		arm_log = LogInfo(msg=f'arm profile: {ARM_PROFILE} not found; skipping it')
	# 비워 두면 넘기지 않는다: 프로파일 → 패키지 설정(비면 호스트 이름) 값을 쓴다
	robot_id = LaunchConfiguration('robot_id').perform(context)
	if robot_id:
		# 문자열로 넘긴다: '04'가 YAML 정수 4가 되지 않게 (follower_arm.launch.py와 같다)
		parameters.append({'robot_id': ParameterValue(robot_id, value_type=str)})
	return [arm_log, make_node(
		package='lekiwi_lerobot',
		executable='lekiwi_follower_arm_node',
		name='lekiwi_follower_arm_node',
		output='screen',
		prefix=lerobot_python,
		parameters=parameters,
	)]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			'robot_profile', default_value='',
			description='장비별 설정 파일. 비우면 ~/lekiwi_ws/robot_profile.yaml, 없으면 패키지 기본값'),
		DeclareLaunchArgument('use_imu', default_value='true', description='BMI160 IMU'),
		DeclareLaunchArgument('use_lidar', default_value='true', description='YDLIDAR와 자기 반사 필터'),
		DeclareLaunchArgument(
			'use_camera', default_value='false', description='전방 · 손목 카메라 (lekiwi_camera가 빌드돼 있어야 함)'),
		DeclareLaunchArgument(
			'use_motors', default_value='true', description='모터 버스 드라이버 (모터 9개 읽기)'),
		DeclareLaunchArgument(
			'use_base_controller', default_value='true',
			description='바퀴 쓰기 + 베이스 컨트롤러. false면 모터는 읽기 전용'),
		DeclareLaunchArgument(
			'use_arm', default_value='true',
			description='드라이버 팔 쓰기 + 팔로워 팔(lekiwi_lerobot, 설치돼 있을 때만). 팔 토크는 바꾸지 않는다'),
		DeclareLaunchArgument(
			'lerobot_python', default_value=LEROBOT_PYTHON,
			description='팔로워 팔 노드를 실행할 lerobot 0.6.0 가상환경 python. 없으면 팔로워를 건너뛴다'),
		DeclareLaunchArgument(
			'robot_id', default_value='',
			description='팔로워 보정 파일 이름 <robot_id>.json. 비우면 장비 프로파일 · 팔로워 설정(호스트 이름)을 쓴다'),
		DeclareLaunchArgument(
			'joint_state_rate', default_value='10',
			description='[Hz] 관절 상태 · 관절 TF 발행 주기 (화면 표시용). Pi에서 Nav2를 함께 돌리므로 낮게 둔다'),
		OpaqueFunction(function=launch_setup),
	])
