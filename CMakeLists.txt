cmake_minimum_required(VERSION 3.5)
project(ros2_whill)

#add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "WhillController")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME} src/main.cpp src/whill_modelc/com_whill.cpp src/whill_modelc/uart.cpp src/odom.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  params
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()