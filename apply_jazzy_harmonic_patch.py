#!/usr/bin/env python3
from __future__ import annotations

import argparse
import difflib
import re
import shutil
import sys
from pathlib import Path

TEXT_SUFFIXES = {
    ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp",
    ".xml", ".sdf", ".xmacro", ".py", ".yaml", ".yml",
    ".cmake", ".md", ".sh", ".txt", ".in", ".config", ".qml", ".qrc", ".repos",
}

LITERAL_REPLACEMENTS = [
    ("gz/sim/", "gz/sim/"),
    ("gz::sim::", "gz::sim::"),
    ("gz::sim", "gz::sim"),
    ("GZ_SIM_VISIBLE", "GZ_SIM_VISIBLE"),
    ("GZ_SIM_SYSTEMS_", "GZ_SIM_SYSTEMS_"),
    ("GZ_SIM_", "GZ_SIM_"),
    ("gz/transport/", "gz/transport/"),
    ("gz/msgs/", "gz/msgs/"),
    ("gz/math/", "gz/math/"),
    ("gz/common/", "gz/common/"),
    ("gz/plugin/", "gz/plugin/"),
    ("gz/gui/", "gz/gui/"),
    ("gz::transport::", "gz::transport::"),
    ("gz::msgs::", "gz::msgs::"),
    ("gz::math::", "gz::math::"),
    ("gz::common::", "gz::common::"),
    ("gz::plugin::", "gz::plugin::"),
    ("gz::gui::", "gz::gui::"),
    ("using namespace gz;", "using namespace gz;"),
    ("namespace gz", "namespace gz"),
    ("GZ_ADD_PLUGIN_ALIAS", "GZ_ADD_PLUGIN_ALIAS"),
    ("GZ_ADD_PLUGIN", "GZ_ADD_PLUGIN"),
    ("gzerr", "gzerr"),
    ("gzwarn", "gzwarn"),
    ("gzmsg", "gzmsg"),
    ("gzdbg", "gzdbg"),
    ("gz.msgs.", "gz.msgs."),
    ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"),
    ("GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH"),
    ("GZ_GUI_PLUGIN_PATH", "GZ_GUI_PLUGIN_PATH"),
    ("GZ_FILE_PATH", "GZ_FILE_PATH"),
    ("<gz-gui>", "<gz-gui>"),
    ("</gz-gui>", "</gz-gui>"),
    ("ros_gz_bridge", "ros_gz_bridge"),
    ("ros_gz_sim", "ros_gz_sim"),
]

REGEX_REPLACEMENTS = [
    (r"set\s*\(\s*CMAKE_CXX_STANDARD\s+14\s*\)", "set(CMAKE_CXX_STANDARD 17)"),
    (r"find_package\(ignition-gazebo6\s+REQUIRED\)", "find_package(gz-sim8 REQUIRED)"),
    (r"find_package\(ignition-msgs8\s+REQUIRED\)", "find_package(gz-msgs10 REQUIRED)"),
    (r"find_package\(ignition-transport11\s+REQUIRED\)", "find_package(gz-transport13 REQUIRED)"),
    (r"find_package\(ignition-gui6\s+REQUIRED\)", "find_package(gz-gui8 REQUIRED)"),
    (r"find_package\(ignition-common4\s+REQUIRED\)", "find_package(gz-common5 REQUIRED)"),
    (r"find_package\(gz-transport12\s+REQUIRED\)", "find_package(gz-transport13 REQUIRED)"),
    (r"set\(GZ_SIM_VER\s+6\)", "set(GZ_SIM_VER 8)"),
    (r"set\(GZ_MSGS_VER\s+8\)", "set(GZ_MSGS_VER 10)"),
    (r"set\(GZ_TRANSPORT_VER\s+11\)", "set(GZ_TRANSPORT_VER 13)"),
    (r"set\(GZ_TRANSPORT_VER\s+\$\{ignition-transport11_VERSION_MAJOR\}\)",
     "set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})"),
    (r"set\(GZ_MSGS_VER\s+\$\{ignition-msgs8_VERSION_MAJOR\}\)",
     "set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})"),
    (r"ignition-gazebo\$\{GZ_SIM_VER\}::ignition-gazebo\$\{GZ_SIM_VER\}",
     "gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}"),
    (r"ignition-msgs\$\{GZ_MSGS_VER\}::core", "gz-msgs${GZ_MSGS_VER}::core"),
    (r"ignition-transport\$\{GZ_TRANSPORT_VER\}::core", "gz-transport${GZ_TRANSPORT_VER}::core"),
    (r"gz-transport13::gz-transport13", "gz-transport13::gz-transport13"),
    (r"ament_export_dependencies\(ignition-msgs\$\{GZ_MSGS_VER\}\)",
     "ament_export_dependencies(gz-msgs${GZ_MSGS_VER})"),
    (r"ament_export_dependencies\(ignition-transport\$\{GZ_TRANSPORT_VER\}\)",
     "ament_export_dependencies(gz-transport${GZ_TRANSPORT_VER})"),
    (r"<depend>gz-msgs10</depend>", "<depend>gz-msgs10</depend>"),
    (r"<depend>gz-transport13</depend>", "<depend>gz-transport13</depend>"),
    (r"<depend>gz-sim8</depend>", "<depend>gz-sim8</depend>"),
    (r"<depend>gz-gui8</depend>", "<depend>gz-gui8</depend>"),
    (r"<depend>gz-common5</depend>", "<depend>gz-common5</depend>"),
    (r"<depend>gz-transport13</depend>", "<depend>gz-transport13</depend>"),
    (r'Compiling against Gazebo Harmonic', 'Compiling against Gazebo Harmonic'),
    (r'(["\']gz_version["\']\s*:\s*)["\']6["\']', r'\1"8"'),
    (r'(["\'])ign\1(\s*,)', r'\1gz\1\2'),
]

RMOSS_GZ_PLUGINS_CMAKE = '''cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER 10)
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER 13)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)

add_library(MecanumDrive2 SHARED plugins/mecanum_drive2/MecanumDrive2.cc)
target_link_libraries(MecanumDrive2
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(ProjectileShooter SHARED plugins/projectile_shooter/ProjectileShooter.cc)
target_link_libraries(ProjectileShooter
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(LightBarController SHARED plugins/light_bar_controller/LightBarController.cc)
target_link_libraries(LightBarController
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(TARGETS MecanumDrive2 ProjectileShooter LightBarController
  DESTINATION plugins
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")
ament_package()
'''

RMOSS_GZ_BASE_CMAKE = '''cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Harmonic transport version ${GZ_TRANSPORT_VER}")

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/gz_module/gz_chassis_actuator.cpp
  src/gz_module/gz_gimbal_actuator.cpp
  src/gz_module/gz_shoot_actuator.cpp
  src/gz_module/gz_gimbal_imu.cpp
  src/gz_module/gz_gimbal_encoder.cpp
  src/gz_module/gz_odometry.cpp
  src/gz_module/gz_light_bar_cmd.cpp
  src/pid.cpp
  src/chassis_controller.cpp
  src/gimbal_controller.cpp
  src/shooter_controller.cpp
  src/odometry_publisher.cpp
  src/rmua19_robot_base_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rmoss_interfaces
  tf2_ros
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
  gz-msgs${GZ_MSGS_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::core
)

add_executable(rmua19_robot_base src/rmua19_robot_base_main.cpp)
target_link_libraries(rmua19_robot_base ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_gz_base::Rmua19RobotBaseNode")

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS rmua19_robot_base DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/test_chassis_cmd.py
  scripts/test_gimbal_cmd.py
  scripts/test_gimbal_cmd2.py
  scripts/test_shoot_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(gz-msgs${GZ_MSGS_VER})
ament_export_dependencies(gz-transport${GZ_TRANSPORT_VER})
ament_package()
'''

RMOSS_GZ_BRIDGE_CMAKE = '''cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Harmonic transport version ${GZ_TRANSPORT_VER}")

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rmoss_interfaces
  tf2_ros
)

add_executable(pose_bridge src/pose_bridge_node.cpp src/pose_bridge_main.cpp)
target_link_libraries(pose_bridge
  gz-msgs${GZ_MSGS_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::core
)
ament_target_dependencies(pose_bridge ${dependencies})
ament_target_dependencies(pose_bridge ros_gz_bridge)

# ECUST-specific extension retained.
add_executable(rfid_bridge src/rfid_bridge_node.cpp src/rfid_bridge_main.cpp)
target_link_libraries(rfid_bridge
  gz-msgs${GZ_MSGS_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::core
)
ament_target_dependencies(rfid_bridge ${dependencies})
ament_target_dependencies(rfid_bridge ros_gz_bridge)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS pose_bridge rfid_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_dependencies(gz-msgs${GZ_MSGS_VER})
ament_export_dependencies(gz-transport${GZ_TRANSPORT_VER})
ament_package()
'''

GUI_PLUGINS_CMAKE = '''cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

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
'''

AUTHORITATIVE_FILES = {
    "src/rmoss_gazebo/rmoss_gz_plugins/CMakeLists.txt": RMOSS_GZ_PLUGINS_CMAKE,
    "src/rmoss_gazebo/rmoss_gz_base/CMakeLists.txt": RMOSS_GZ_BASE_CMAKE,
    "src/rmoss_gazebo/rmoss_gz_bridge/CMakeLists.txt": RMOSS_GZ_BRIDGE_CMAKE,
    "src/gui_plugins/CMakeLists.txt": GUI_PLUGINS_CMAKE,
    "src/rmoss_gazebo/rmoss_gz_plugins/env-hooks/gazebo.dsv.in":
        "prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;plugins\n",
    "src/gui_plugins/env-hooks/gazebo.dsv.in":
        "prepend-non-duplicate;GZ_GUI_PLUGIN_PATH;plugins\n",
}

def is_text_candidate(path: Path) -> bool:
    if path.name == "CMakeLists.txt" or path.name.endswith(".dsv.in"):
        return True
    return path.suffix.lower() in TEXT_SUFFIXES

def migrate_standard_system_plugin_filenames(text: str) -> str:
    def repl(match: re.Match) -> str:
        body = match.group(1)
        return f'filename="gz-sim-{body}-system.so"'
    return re.sub(
        r'filename="(?:lib)?ignition-gazebo-([A-Za-z0-9_-]+?)-system(?:\.so)?"',
        repl,
        text,
    )

def transform_text(path: Path, text: str) -> str:
    new = migrate_standard_system_plugin_filenames(text)

    for old, replacement in LITERAL_REPLACEMENTS:
        new = new.replace(old, replacement)

    for pattern, replacement in REGEX_REPLACEMENTS:
        new = re.sub(pattern, replacement, new)

    posix = path.as_posix()
    if "/rmoss_gz_plugins/plugins/" in posix:
        new = re.sub(r"\bnamespace gazebo\b", "namespace sim", new)
        new = new.replace("using namespace gazebo;", "using namespace sim;")

    new = new.replace("gz::", "gz::")
    new = new.replace('#include "gz/', '#include "gz/')
    new = new.replace("#include <gz/", "#include <gz/")
    new = new.replace("GZ_MSGS_VER", "GZ_MSGS_VER")
    new = new.replace("GZ_TRANSPORT_VER", "GZ_TRANSPORT_VER")
    new = new.replace("GZ_SIM_VER", "GZ_SIM_VER")
    return new

def strip_legacy_gazebo_plugin_path_block(text: str) -> str:
    pattern = re.compile(
        r'\n\s*append_enviroment_worlds\s*=\s*AppendEnvironmentVariable\(\s*'
        r'name="GAZEBO_PLUGIN_PATH",\s*'
        r'value=os\.path\.join\(pkg_simulator,\s*"resource",\s*"worlds"\),\s*'
        r'\)\s*\n',
        re.MULTILINE,
    )
    text = pattern.sub("\n", text)
    text = re.sub(
        r'^\s*ld\.add_action\(append_enviroment_worlds\)\s*\n',
        "",
        text,
        flags=re.MULTILINE,
    )
    return text

def write_text(path: Path, new: str, dry_run: bool) -> None:
    if not dry_run:
        path.write_text(new, encoding="utf-8")

def validate_root(root: Path) -> None:
    required = [
        root / "src/rmoss_gazebo/rmoss_gz_plugins",
        root / "src/rmu_gazebo_simulator",
    ]
    missing = [p for p in required if not p.exists()]
    if missing:
        raise SystemExit("Not gazebo_simulator_origin; missing: " + ", ".join(map(str, missing)))

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--show-diff", action="store_true")
    args = parser.parse_args()

    root = Path(args.repo).resolve()
    validate_root(root)

    changed = []
    diffs = []

    for path in sorted(root.rglob("*")):
        if not path.is_file() or not is_text_candidate(path):
            continue
        if any(part in {".git", "build", "install", "log"} for part in path.parts):
            continue

        old = path.read_text(encoding="utf-8")
        new = transform_text(path, old)

        if path.name == "gazebo.launch.py":
            new = strip_legacy_gazebo_plugin_path_block(new)
            new = new.replace("ign_config_path", "gz_config_path")

        if path.name == "spawn_robots.launch.py":
            new = new.replace("robot_ign_bridge", "robot_gz_bridge")

        if new != old:
            rel = path.relative_to(root).as_posix()
            changed.append(rel)
            if args.show_diff:
                diffs.extend(difflib.unified_diff(
                    old.splitlines(True), new.splitlines(True),
                    fromfile=f"a/{rel}", tofile=f"b/{rel}"
                ))
            write_text(path, new, args.dry_run)

    for rel, content in AUTHORITATIVE_FILES.items():
        path = root / rel
        if not path.exists():
            print(f"WARN missing expected file: {rel}", file=sys.stderr)
            continue
        old = path.read_text(encoding="utf-8")
        if old != content:
            changed.append(rel)
            if args.show_diff:
                diffs.extend(difflib.unified_diff(
                    old.splitlines(True), content.splitlines(True),
                    fromfile=f"a/{rel}", tofile=f"b/{rel}"
                ))
            write_text(path, content, args.dry_run)

    # Move stale GUI config directory name.
    src = root / "src/rmu_gazebo_simulator/resource/ign"
    dst = root / "src/rmu_gazebo_simulator/resource/gz"
    if src.exists():
        changed.append("src/rmu_gazebo_simulator/resource/ign -> resource/gz")
        if not args.dry_run:
            if dst.exists():
                for item in src.iterdir():
                    target = dst / item.name
                    if target.exists():
                        raise RuntimeError(f"Refusing to overwrite {target}")
                    shutil.move(str(item), str(target))
                src.rmdir()
            else:
                src.rename(dst)

    if args.show_diff and diffs:
        print("".join(diffs))

    unique = list(dict.fromkeys(changed))
    state = "DRY RUN" if args.dry_run else "APPLIED"
    print(f"[{state}] touched {len(unique)} paths")
    for rel in unique:
        print("  " + rel)

    print("\nValidation:")
    print("  rm -rf build install log")
    print("  source /opt/ros/jazzy/setup.bash")
    print("  rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy")
    print("  colcon build --symlink-install")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
