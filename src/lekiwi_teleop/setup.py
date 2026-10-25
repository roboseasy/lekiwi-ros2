from setuptools import setup

package_name = "lekiwi_teleop"

setup(
	name=package_name,
	version="0.0.1",
	packages=[package_name],
	data_files=[
		("share/ament_index/resource_index/packages", ["resource/" + package_name]),
		("share/" + package_name, ["package.xml"]),
	],
	install_requires=["setuptools"],
	# colcon test가 unittest 대신 pytest로 함수형 테스트까지 돌리게 한다
	extras_require={"test": ["pytest"]},
	zip_safe=True,
	maintainer="khw",
	maintainer_email="khw11044@gmail.com",
	description="LeKiwi 베이스를 조작 · 시험하는 도구 (조작 창 · 터미널 teleop, 짧은 cmd_vel, Nav2 모터 감독, 거리 시험, odom 보정 계산)",
	license="Apache-2.0",
	entry_points={
		"console_scripts": [
			"lekiwi_base_teleop = lekiwi_teleop.base_teleop:main",
			"lekiwi_guarded_base_teleop = lekiwi_teleop.guarded_base_teleop:main",
			"lekiwi_terminal_teleop = lekiwi_teleop.terminal_teleop:main",
			"lekiwi_keyboard_teleop = lekiwi_teleop.keyboard_teleop:main",
			"lekiwi_odom_calibration = lekiwi_teleop.odom_calibration:main",
			"lekiwi_distance_trial = lekiwi_teleop.distance_trial:main",
			"lekiwi_safe_cmd_vel = lekiwi_teleop.safe_cmd_vel:main",
			"lekiwi_nav2_motor_guard = lekiwi_teleop.nav2_motor_guard:main",
		],
	},
)
