cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gui_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
find_package(ament_cmake REQUIRED)

# Gazebo Harmonic / gz-gui8, following the upstream gz-gui8 plugin example.
find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")

find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

QT5_ADD_RESOURCES(resources_RCC
  plugins/referee_ui/referee.qrc
  plugins/race_control/racecontrol.qrc
)

add_library(RefereeUI SHARED
  plugins/referee_ui/RefereeUI.cc
  ${resources_RCC}
)
target_link_libraries(RefereeUI
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)
install(TARGETS RefereeUI DESTINATION plugins)

add_library(RaceControl SHARED
  plugins/race_control/RaceControl.cc
  ${resources_RCC}
)
target_link_libraries(RaceControl
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(RaceControl
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS RaceControl DESTINATION plugins)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")
ament_package()
