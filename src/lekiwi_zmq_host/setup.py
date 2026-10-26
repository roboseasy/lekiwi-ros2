import os
from glob import glob

from setuptools import setup

package_name = 'lekiwi_zmq_host'

setup(
	name=package_name,
	version='0.0.1',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'config'),
			glob('config/*.yaml')),
	],
	install_requires=['setuptools'],
	# colcon test가 unittest 대신 pytest로 함수형 테스트까지 돌리게 한다
	extras_require={'test': ['pytest']},
	zip_safe=True,
	maintainer='khw',
	maintainer_email='khw11044@gmail.com',
	description='lerobot LeKiwi 호스트 프로토콜(ZMQ)을 ROS 토픽 · 서비스로 바꾸는 노드',
	license='Apache-2.0',
	entry_points={
		'console_scripts': [
			'lekiwi_zmq_host_node = lekiwi_zmq_host.node:main',
		],
	},
)
