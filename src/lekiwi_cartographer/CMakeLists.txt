cmake_minimum_required(VERSION 3.8)
project(lekiwi_cartographer)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

# 지도 저장: ros2 run lekiwi_cartographer save_map.sh
install(PROGRAMS scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # cartographer_ros 없이 설정 · 지도 저장 스크립트만 검사한다
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cartographer_contract test/test_cartographer_contract.py
    ENV PYTHONDONTWRITEBYTECODE=1
  )
  ament_add_pytest_test(test_save_map test/test_save_map.py
    TIMEOUT 120
    ENV PYTHONDONTWRITEBYTECODE=1
  )
endif()

ament_package()
