import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_base_controller')
    default_params = os.path.join(package_share, 'config', 'lekiwi_base_controller.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='lekiwi_base_controller_node 파라미터 파일',
        ),
        # 바퀴 버스는 lekiwi_motor_driver_node가 연다. 이 노드는 드라이버 토픽 · 서비스만 쓴다.
        Node(
            package='lekiwi_base_controller',
            executable='lekiwi_base_controller_node',
            name='lekiwi_base_controller_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
