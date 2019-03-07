cmake_minimum_required(VERSION 3.5)
project(ros2_whill)

add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy  REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_whill_interfaces REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
	${rclcpp_INCLUDE_DIRS}
	${rmw_implementation_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	${ros2_whill_interfaces_INCLUDE_DIRS}
)
message(WARNING "ros2_whill INC: ${ros2_whill_interfaces_INCLUDE_DIRS}")

#add_executable(whill_modelc_publisher
#	src/whill_modelc_publisher.cpp 
#	src/whill_modelc/uart.cpp 
#	src/whill_modelc/com_whill.cpp 
#	src/odom.cpp
#)

#ament_target_dependencies(whill_modelc_publisher
#	"rclcpp"
#)

add_executable(whill_modelc_controller 
	src/whill_modelc_controller.cpp 
	src/whill_modelc/uart.cpp 
	src/whill_modelc/com_whill.cpp
)

ament_target_dependencies(whill_modelc_controller
	"rclcpp"
	"ros2_whill_interfaces"
)

#target_link_libraries(whill_modelc_publisher
#	${rclcpp_LIBRARIES}
#	${rmw_implementation_LIBRARIES}
#	${std_msgs_LIBRARIES}
#)

target_link_libraries(whill_modelc_controller
	${rclcpp_LIBRARIES}
	${rmw_implementation_LIBRARIES}
	${std_msgs_LIBRARIES}
	${sensor_msgs_LIBRARIES}
	${ros2_whill_interfaces_LIBRARIES}
)

#install(TARGETS whill_modelc_publisher RUNTIME DESTINATION bin)
install(TARGETS whill_modelc_controller RUNTIME DESTINATION bin)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
