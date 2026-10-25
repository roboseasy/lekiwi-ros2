from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	use_sim_time = LaunchConfiguration('use_sim_time')
	resolution = LaunchConfiguration('resolution')
	publish_period_sec = LaunchConfiguration('publish_period_sec')

	# Cartographer 서브맵 → /map (OccupancyGrid)
	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='시뮬레이션 시간 사용 여부',
		),
		DeclareLaunchArgument(
			'resolution',
			default_value='0.05',
			description='OccupancyGrid 칸 크기 [m]',
		),
		DeclareLaunchArgument(
			'publish_period_sec',
			default_value='1.0',
			description='OccupancyGrid 발행 주기 [s]',
		),
		Node(
			package='cartographer_ros',
			executable='cartographer_occupancy_grid_node',
			name='cartographer_occupancy_grid_node',
			output='screen',
			parameters=[{
				'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
				'resolution': ParameterValue(resolution, value_type=float),
				'publish_period_sec': ParameterValue(publish_period_sec, value_type=float),
			}],
		),
	])
