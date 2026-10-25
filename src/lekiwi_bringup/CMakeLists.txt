cmake_minimum_required(VERSION 3.8)
project(lekiwi_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # launch 구성 · 장비 프로파일 시험 (하드웨어 · 다른 패키지 설치 없이)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup_launch test/test_bringup_launch.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # 모터 드라이버 + 베이스 컨트롤러를 가상 Feetech 버스에 붙인 전체 흐름 시험 (하드웨어 없음)
  ament_add_pytest_test(test_motor_stack_pty test/test_motor_stack_pty.py
    TIMEOUT 300
  )
endif()

ament_package()
