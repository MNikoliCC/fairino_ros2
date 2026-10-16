cmake_minimum_required(VERSION 3.16)
project(fairino_cartesian_servo LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_oink REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS-independent pieces (unit tested without a ROS graph).
add_library(fairino_servo_core SHARED
  src/fault_catalog.cpp
  src/servo_math.cpp
)
target_include_directories(fairino_servo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fairino_servo_core PUBLIC pinocchio::pinocchio Eigen3::Eigen)

add_executable(fairino_cartesian_servo
  src/main.cpp
  src/cartesian_servo_node.cpp
  src/pointing_task.cpp
)
target_link_libraries(fairino_cartesian_servo
  fairino_servo_core
  roboplan::roboplan
  roboplan::roboplan_filters
  roboplan_oink::roboplan_oink
  pinocchio::pinocchio
  ${controller_manager_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${fairino_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)

add_executable(fairino_fault_catalog src/fault_catalog_tool.cpp)
target_link_libraries(fairino_fault_catalog fairino_servo_core)

install(TARGETS fairino_servo_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS fairino_cartesian_servo fairino_fault_catalog
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fault_catalog test/test_fault_catalog.cpp)
  target_link_libraries(test_fault_catalog fairino_servo_core)
  ament_add_gtest(test_servo_math test/test_servo_math.cpp)
  target_link_libraries(test_servo_math fairino_servo_core)
endif()

ament_package()
