cmake_minimum_required(VERSION 3.8)
project(lekiwi_navigation2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # Nav2 없이 파라미터 · launch 계약만 검사한다
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_contract test/test_navigation_contract.py
    ENV PYTHONDONTWRITEBYTECODE=1
  )
endif()

ament_package()
