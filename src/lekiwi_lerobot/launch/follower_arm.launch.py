"""팔로워 팔 노드(lekiwi_follower_arm_node)를 lerobot이 든 python으로 실행한다.

ROS 설정을 source한 셸에서 prefix로 lerobot_python을 붙인다. rclpy · 메시지는
ROS 경로(PYTHONPATH)에서, lerobot · numpy 2.2는 그 python 환경에서 온다
(2026-10-22 Pi에서 ~/lerobot060_venv로 확인). 모터 버스는 열지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context: LaunchContext) -> list[Node]:
	"""robot_id 인자는 비어 있지 않을 때만 params_file의 값을 덮어쓴다."""
	parameters = [LaunchConfiguration('params_file')]
	robot_id = LaunchConfiguration('robot_id').perform(context)
	if robot_id:
		# 문자열로 넘긴다: '04'가 YAML 정수 4가 되지 않게
		parameters.append(
			{'robot_id': ParameterValue(robot_id, value_type=str)})
	return [
		Node(
			package='lekiwi_lerobot',
			executable='lekiwi_follower_arm_node',
			name='lekiwi_follower_arm_node',
			output='screen',
			prefix=os.path.expanduser(
				LaunchConfiguration('lerobot_python').perform(context)),
			parameters=parameters,
		),
	]


def generate_launch_description() -> LaunchDescription:
	default_params = os.path.join(
		get_package_share_directory('lekiwi_lerobot'),
		'config',
		'follower_arm.yaml',
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'params_file',
			default_value=default_params,
			description='lekiwi_follower_arm_node 파라미터 파일',
		),
		DeclareLaunchArgument(
			'lerobot_python',
			default_value=os.path.expanduser('~/lerobot060_venv/bin/python'),
			description='lerobot 0.6.0이 든 python (venv · conda)',
		),
		DeclareLaunchArgument(
			'robot_id',
			default_value='',
			description=(
				'보정 파일 이름 <robot_id>.json. 비우면 params_file의 값 '
				'(그것도 비면 호스트 이름)'),
		),
		OpaqueFunction(function=launch_setup),
	])
