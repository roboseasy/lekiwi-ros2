import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_lidar')
    default_params = os.path.join(package_share, 'config', 'lekiwi_lidar.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='lekiwi_lidar_node 파라미터 파일',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Tmini Plus 시리얼 포트',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='lekiwi/sensor/scan',
            description='LaserScan 퍼블리시 토픽. SLAM 입력으로 쓰려면 /scan',
        ),
        # 같은 라이다를 쓰는 lekiwi_sensors/ydlidar.launch.py(브링업 use_lidar:=true)와 동시에 실행하지 않는다.
        Node(
            package='lekiwi_lidar',
            executable='lekiwi_lidar_node',
            name='lekiwi_lidar_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'port': port,
                    'scan_topic': scan_topic,
                },
            ],
        ),
    ])
