cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Harmonic transport version ${GZ_TRANSPORT_VER}")

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/gz_cam_node.cpp)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  rmoss_interfaces
  image_transport
  ros_gz_bridge
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
  gz-msgs${GZ_MSGS_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::core
)

add_executable(gz_cam src/gz_cam_main.cpp)
target_link_libraries(gz_cam ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_gz_cam::GzCamNode")

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS gz_cam DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
