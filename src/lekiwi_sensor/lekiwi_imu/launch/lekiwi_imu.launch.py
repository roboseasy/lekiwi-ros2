import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('lekiwi_imu')
    default_params = os.path.join(package_share, 'config', 'lekiwi_imu.yaml')

    params_file = LaunchConfiguration('params_file')
    i2c_address = LaunchConfiguration('i2c_address')
    publish_orientation_tf = LaunchConfiguration('publish_orientation_tf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='lekiwi_imu_node 파라미터 파일',
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='104',
            description='BMI160 7-bit I2C 주소(10진수). 104=0x68, 105=0x69',
        ),
        DeclareLaunchArgument(
            'publish_orientation_tf',
            default_value='false',
            description='RViz 확인용 imu_world -> imu_orientation TF 발행',
        ),
        # 같은 BMI160을 쓰는 lekiwi_sensors/bmi160_node(브링업 use_imu:=true)와 동시에 실행하지 않는다.
        Node(
            package='lekiwi_imu',
            executable='lekiwi_imu_node',
            name='lekiwi_imu_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'i2c_address': ParameterValue(i2c_address, value_type=int),
                    'publish_orientation_tf': ParameterValue(publish_orientation_tf, value_type=bool),
                },
            ],
        ),
    ])
