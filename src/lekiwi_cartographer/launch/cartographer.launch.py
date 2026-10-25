import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	package_share = get_package_share_directory('lekiwi_cartographer')

	cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
	configuration_basename = LaunchConfiguration('configuration_basename')
	scan_topic = LaunchConfiguration('scan_topic')
	imu_topic = LaunchConfiguration('imu_topic')
	use_sim_time = LaunchConfiguration('use_sim_time')
	resolution = LaunchConfiguration('resolution')
	publish_period_sec = LaunchConfiguration('publish_period_sec')
	use_rviz = LaunchConfiguration('use_rviz')

	# Pi에서 bringup과 함께 실행한다. 스캔 필터는 bringup이 띄우므로 여기서 띄우지 않는다.
	# Pi에는 화면이 없어 RViz는 PC에서 따로 연다 (rviz2 -d .../slam.rviz).
	preflight_log = LogInfo(
		msg=[
			'LeKiwi Cartographer 사전 확인: bringup이 odom, ', scan_topic, ', ', imu_topic,
			' (IMU 설정일 때)와 TF odom -> base_footprint -> ... -> lidar_link, imu_link를 ',
			'발행해야 합니다. Cartographer는 map -> odom을 발행합니다. ',
			'같은 ROS domain에서 AMCL(lekiwi_navigation2)과 함께 실행하지 마세요.',
		],
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'cartographer_config_dir',
			default_value=os.path.join(package_share, 'config'),
			description='Cartographer lua 설정 폴더',
		),
		DeclareLaunchArgument(
			'configuration_basename',
			default_value='lekiwi_2d_imu.lua',
			description='Cartographer lua 설정 파일. 표준 조립품은 BMI160을 쓴다. IMU가 없으면 lekiwi_2d.lua',
		),
		DeclareLaunchArgument(
			'scan_topic',
			default_value='lekiwi/sensor/scan_filtered',
			description='Cartographer 입력 scan으로 연결할 LaserScan 토픽 (Pi lekiwi_scan_filter 출력)',
		),
		DeclareLaunchArgument(
			'imu_topic',
			default_value='lekiwi/sensor/offset_imu',
			description='Cartographer 입력 imu로 연결할 sensor_msgs/Imu 토픽 (자이로 바이어스만 제거)',
		),
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
		DeclareLaunchArgument(
			'use_rviz',
			default_value='false',
			description='이 컴퓨터에서 SLAM RViz를 함께 실행할지 여부. Pi에는 화면이 없어 기본은 false (PC에서 따로 연다)',
		),
		preflight_log,
		# odom은 표준 이름 그대로 구독한다.
		Node(
			package='cartographer_ros',
			executable='cartographer_node',
			name='cartographer_node',
			output='screen',
			parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
			arguments=[
				'-configuration_directory', cartographer_config_dir,
				'-configuration_basename', configuration_basename,
			],
			remappings=[('scan', scan_topic), ('imu', imu_topic)],
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(package_share, 'launch', 'occupancy_grid.launch.py')),
			launch_arguments={
				'use_sim_time': use_sim_time,
				'resolution': resolution,
				'publish_period_sec': publish_period_sec,
			}.items(),
		),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d', os.path.join(package_share, 'rviz', 'slam.rviz')],
			condition=IfCondition(use_rviz),
		),
	])
