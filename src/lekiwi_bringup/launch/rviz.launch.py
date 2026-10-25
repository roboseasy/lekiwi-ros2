"""PC에서 실행 중인 로봇을 RViz로 본다 (로봇 모델 · TF · odom · 걸러진 스캔).

Pi에서 bringup.launch.py가 돌고, PC와 Pi의 ROS_DOMAIN_ID가 같아야 한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			'rviz_config',
			default_value=PathJoinSubstitution([FindPackageShare('lekiwi_description'), 'rviz', 'base.rviz']),
			description='RViz 설정 파일'),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d', LaunchConfiguration('rviz_config')],
		),
	])
