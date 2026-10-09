#!/bin/bash

source install/setup.bash

# 强制使用 NVIDIA 独显进行 PRIME Render Offload
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
