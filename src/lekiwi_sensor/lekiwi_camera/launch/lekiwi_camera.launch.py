import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_camera')
    default_params = os.path.join(package_share, 'config', 'lekiwi_camera.yaml')

    params_file = LaunchConfiguration('params_file')
    use_front = LaunchConfiguration('use_front')
    use_wrist = LaunchConfiguration('use_wrist')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='lekiwi_camera 파라미터 파일 (front_camera_node, wrist_camera_node)',
        ),
        DeclareLaunchArgument(
            'use_front',
            default_value='true',
            description='전방 카메라 실행',
        ),
        DeclareLaunchArgument(
            'use_wrist',
            default_value='true',
            description='손목 카메라 실행',
        ),
        # 카메라마다 노드를 하나씩 띄운다. 같은 카메라를 쓰는 LeRobot 호스트나
        # lekiwi_sensors/cameras.launch.py(usb_cam)와 동시에 실행하지 않는다.
        Node(
            package='lekiwi_camera',
            executable='lekiwi_camera_node',
            name='front_camera_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_front),
        ),
        Node(
            package='lekiwi_camera',
            executable='lekiwi_camera_node',
            name='wrist_camera_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_wrist),
        ),
    ])
