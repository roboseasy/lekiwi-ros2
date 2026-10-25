import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_motor_driver')
    default_params = os.path.join(package_share, 'config', 'lekiwi_motor_driver.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    wheel_write_enabled = LaunchConfiguration('wheel_write_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='lekiwi_motor_driver_node 파라미터 파일',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Feetech 버스 시리얼 포트',
        ),
        DeclareLaunchArgument(
            'wheel_write_enabled',
            default_value='false',
            description='바퀴 속도 · 토크 쓰기를 연다. false면 읽기 전용',
        ),
        # 같은 버스를 쓰는 LeKiwi lekiwi_node(브링업)나 LeRobot 호스트와 동시에 실행하지 않는다.
        Node(
            package='lekiwi_motor_driver',
            executable='lekiwi_motor_driver_node',
            name='lekiwi_motor_driver_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'port': port,
                    'wheel_write_enabled': ParameterValue(wheel_write_enabled, value_type=bool),
                },
            ],
        ),
    ])
