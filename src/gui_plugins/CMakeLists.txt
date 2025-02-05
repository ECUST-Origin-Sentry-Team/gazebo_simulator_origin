cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gui_plugins)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

# Find Qt5
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)
find_package(ament_cmake REQUIRED)

# Find the Gazebo gui library
find_package(ignition-gui6 REQUIRED)
find_package(ignition-common4 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-GUI_CXX_FLAGS}")

QT5_ADD_RESOURCES(resources_RCC plugins/referee_ui/referee.qrc)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARY_DIRS}
)

# Generate examples
add_library(RefereeUI SHARED
  plugins/referee_ui/RefereeUI.cc
  ${resources_RCC}
)




target_link_libraries(RefereeUI
${IGNITION-GUI_LIBRARIES}
${Qt5Core_LIBRARIES}
${Qt5Qml_LIBRARIES}
${Qt5Quick_LIBRARIES}
${Qt5QuickControls2_LIBRARIES}
)

install(TARGETS RefereeUI
    DESTINATION plugins
    )

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")


ament_package()