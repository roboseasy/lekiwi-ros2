import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	package_share = get_package_share_directory('lekiwi_scan_filter')
	default_params = os.path.join(package_share, 'config', 'lekiwi_scan_filter.yaml')

	params_file = LaunchConfiguration('params_file')
	use_sim_time = LaunchConfiguration('use_sim_time')

	return LaunchDescription([
		DeclareLaunchArgument(
			'params_file',
			default_value=default_params,
			description='lekiwi_scan_filter_node 파라미터 파일',
		),
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='시뮬레이션 시간 사용 여부',
		),
		# 필터가 끝나면 launch 전체를 끝낸다. 스캔 없이 다른 노드만 남지 않게 한다.
		Node(
			package='lekiwi_scan_filter',
			executable='lekiwi_scan_filter_node',
			name='lekiwi_scan_filter_node',
			output='screen',
			on_exit=[Shutdown(reason='lekiwi_scan_filter_node exited')],
			parameters=[
				params_file,
				{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
			],
		),
	])
