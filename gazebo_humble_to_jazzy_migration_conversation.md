# Gazebo Simulator Origin：Humble → Jazzy / Gazebo Fortress → Harmonic 迁移讨论

> 整理自本次对话中的用户问题与助手回答。  
> 不包含内部工具调用、系统信息或隐藏推理。

---

## 用户

`https://github.com/ECUST-Origin-Sentry-Team/gazebo_simulator_origin` 这个项目从 humble 迁移到 jazzy，gazebo 的版本变了，需要做哪些改动

---

## 助手

目标栈应从：

- ROS 2 Humble
- Gazebo Fortress

迁移到：

- ROS 2 Jazzy
- Gazebo Harmonic

Gazebo Harmonic 对应的主要库版本包括：

- `gz-sim8`
- `gz-msgs10`
- `gz-transport13`
- `gz-gui8`
- `gz-common5`

这个项目当前已经有一部分使用了 `ros_gz`，但底层大量代码仍停留在 Fortress / Ignition API，因此主要需要改以下几类内容。

### 1. CMake 依赖

例如当前：

```cmake
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
```

需要迁移到 Harmonic：

```cmake
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sim8 REQUIRED)
```

对应链接：

```cmake
ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
```

改为：

```cmake
gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
```

建议同时把仍使用 C++14 的 Gazebo 相关模块统一到 C++17。

### 2. C++ API 从 Ignition 改成 Gazebo `gz::*`

例如：

```cpp
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>

namespace ignition {
namespace gazebo {
namespace systems {
```

迁移为：

```cpp
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

namespace gz {
namespace sim {
namespace systems {
```

类型同步迁移：

```cpp
ignition::gazebo::System
ignition::gazebo::UpdateInfo
ignition::transport::Node
ignition::msgs::Twist
ignition::math::PID
```

改为：

```cpp
gz::sim::System
gz::sim::UpdateInfo
gz::transport::Node
gz::msgs::Twist
gz::math::PID
```

日志宏：

```cpp
ignerr
ignwarn
ignmsg
igndbg
```

改为：

```cpp
gzerr
gzwarn
gzmsg
gzdbg
```

插件注册：

```cpp
IGNITION_ADD_PLUGIN(...)
IGNITION_ADD_PLUGIN_ALIAS(...)
```

改为：

```cpp
GZ_ADD_PLUGIN(...)
GZ_ADD_PLUGIN_ALIAS(...)
```

### 3. Launch 文件

当前 `gazebo.launch.py` 中还固定：

```python
"gz_version": "6"
```

这是 Fortress 对应的 Gazebo Sim 6。

迁移 Jazzy / Harmonic 后应使用 Sim 8：

```python
"gz_version": "8"
```

或者直接移除版本固定，让 Jazzy 的 `ros_gz_sim` 使用默认 Gazebo 版本。

还需要修改：

```text
IGN_GAZEBO_RESOURCE_PATH
```

为：

```text
GZ_SIM_RESOURCE_PATH
```

旧的：

```text
GAZEBO_PLUGIN_PATH
```

如果是 Gazebo Classic 遗留用途，应检查是否已经无效；如果目的是 Gazebo Sim system plugin，应使用：

```text
GZ_SIM_SYSTEM_PLUGIN_PATH
```

### 4. `ros_gz_bridge.yaml`

当前 Gazebo message type 仍然有：

```yaml
gz_type_name: "ignition.msgs.Odometry"
gz_type_name: "ignition.msgs.Model"
gz_type_name: "ignition.msgs.Image"
gz_type_name: "ignition.msgs.PointCloudPacked"
gz_type_name: "ignition.msgs.IMU"
gz_type_name: "ignition.msgs.CameraInfo"
```

应改为：

```yaml
gz_type_name: "gz.msgs.Odometry"
gz_type_name: "gz.msgs.Model"
gz_type_name: "gz.msgs.Image"
gz_type_name: "gz.msgs.PointCloudPacked"
gz_type_name: "gz.msgs.IMU"
gz_type_name: "gz.msgs.CameraInfo"
```

### 5. 环境变量

典型迁移：

```text
IGN_GAZEBO_RESOURCE_PATH
→ GZ_SIM_RESOURCE_PATH

IGN_GAZEBO_SYSTEM_PLUGIN_PATH
→ GZ_SIM_SYSTEM_PLUGIN_PATH

IGN_GUI_PLUGIN_PATH
→ GZ_GUI_PLUGIN_PATH
```

### 6. SDF / World plugin

旧 Fortress：

```xml
<plugin
  filename="libignition-gazebo-physics-system.so"
  name="ignition::gazebo::systems::Physics"/>
```

迁移 Harmonic：

```xml
<plugin
  filename="gz-sim-physics-system.so"
  name="gz::sim::systems::Physics"/>
```

类似需要修改：

```text
Physics
SceneBroadcaster
UserCommands
Sensors
Imu
JointController
JointStatePublisher
PerformerDetector
```

自定义插件 filename 可以保持，例如：

```xml
<plugin
  filename="MecanumDrive2"
  name="gz::sim::systems::MecanumDrive2"/>
```

但 C++ 中的 plugin alias 必须同步改成 `gz::sim::systems::*`。

### 7. GUI plugin

当前 GUI CMake 仍有：

```cmake
find_package(ignition-gui6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(gz-transport12 REQUIRED)
```

应迁移到 Harmonic 对应版本：

```cmake
find_package(gz-gui8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
```

GUI 配置中的：

```xml
<ignition-gui>
```

应迁移为：

```xml
<gz-gui>
```

目录名 `resource/ign` 也建议改为：

```text
resource/gz
```

### 8. 推荐构建顺序

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select \
  rmoss_gz_plugins \
  rmoss_gz_base \
  rmoss_gz_bridge \
  rmoss_gz_cam

colcon build --symlink-install \
  --packages-select gui_plugins

gz sim -v 4 path/to/empty_world.sdf

ros2 launch rmu_gazebo_simulator gazebo.launch.py
```

---

## 用户

参考这个 gazebo 版本做一些修改呢  
`https://github.com/robomaster-oss/rmoss_gazebo`

---

## 助手

可以。参考 `robomaster-oss/rmoss_gazebo` 的 **`jazzy` 分支**来改，比单纯按 Gazebo 官方迁移文档更适合你这个项目，因为你仓库里的 `rmoss_gz_base / bridge / cam / plugins` 本身就是从这套代码演化来的。

对比该仓库的 `humble` 和 `jazzy` 分支可以发现，上游的 Jazzy 迁移采用：

```text
ROS 2 Jazzy
Gazebo Harmonic
gz-sim8
gz-msgs10
gz-transport13
C++17
ros_gz
```

并且上游 `humble → jazzy` 的迁移集中在：

```text
rmoss_gz_base
rmoss_gz_bridge
rmoss_gz_cam
rmoss_gz_plugins
```

所以这个项目可以直接把上游 Jazzy 作为迁移模板。

### 1. `rmoss_gz_plugins`

上游 Jazzy 当前使用：

```cmake
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER 10)

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER 13)

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)
```

链接：

```cmake
target_link_libraries(MecanumDrive2
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_link_libraries(ProjectileShooter
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_link_libraries(LightBarController
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
```

因此：

```text
src/rmoss_gazebo/rmoss_gz_plugins/
```

建议直接以上游 `jazzy/rmoss_gz_plugins` 为迁移模板。

### 2. 三个自定义 Gazebo plugin

需要重点迁移：

```text
MecanumDrive2
ProjectileShooter
LightBarController
```

例如 `MecanumDrive2.hh`：

```cpp
#include <gz/sim/System.hh>

namespace gz {
namespace sim {
namespace systems {

class GZ_SIM_VISIBLE MecanumDrive2
    : public gz::sim::System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
```

`.cc` 使用：

```cpp
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
```

插件注册：

```cpp
GZ_ADD_PLUGIN(
    MecanumDrive2,
    gz::sim::System,
    MecanumDrive2::ISystemConfigure,
    MecanumDrive2::ISystemPreUpdate,
    MecanumDrive2::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    MecanumDrive2,
    "gz::sim::systems::MecanumDrive2")
```

### 3. `rmoss_gz_base`

CMake 改成：

```cmake
set(CMAKE_CXX_STANDARD 17)

find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
```

链接：

```cmake
target_link_libraries(${PROJECT_NAME}
    gz-msgs${GZ_MSGS_VER}::core
    gz-transport${GZ_TRANSPORT_VER}::core
)
```

导出依赖：

```cmake
ament_export_dependencies(gz-msgs${GZ_MSGS_VER})
ament_export_dependencies(gz-transport${GZ_TRANSPORT_VER})
```

代码中的：

```cpp
ignition::transport::Node
ignition::msgs::*
ignition::math::PID
```

同步改为：

```cpp
gz::transport::Node
gz::msgs::*
gz::math::PID
```

### 4. `rmoss_gz_cam`

上游 Jazzy 使用：

```cmake
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
```

头文件：

```cpp
#include "gz/msgs/image.pb.h"
#include "gz/transport/Node.hh"
```

callback：

```cpp
void gz_image_cb(const gz::msgs::Image & msg);
```

CameraInfo bridge：

```cpp
std::function<void(const gz::msgs::CameraInfo & msg)> camera_info_cb =
  [&](const gz::msgs::CameraInfo & msg)
  {
    ros_gz_bridge::convert_gz_to_ros(msg, cam_info_);
  };
```

### 5. `rmoss_gz_bridge`

上游 Jazzy 主要有 `pose_bridge`。

但你们项目额外还有：

```text
rfid_bridge
```

所以不能整个目录直接覆盖。

CMake 应改为：

```cmake
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
```

`pose_bridge` 和 `rfid_bridge` 都链接：

```cmake
gz-msgs${GZ_MSGS_VER}::core
gz-transport${GZ_TRANSPORT_VER}::core
```

`rfid_bridge` 中所有：

```cpp
ignition::transport
ignition::msgs
```

同步改成：

```cpp
gz::transport
gz::msgs
```

### 6. env hook

上游 Jazzy：

```text
prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;plugins
```

所以：

```text
IGN_GAZEBO_SYSTEM_PLUGIN_PATH
→ GZ_SIM_SYSTEM_PLUGIN_PATH

IGN_GAZEBO_RESOURCE_PATH
→ GZ_SIM_RESOURCE_PATH

IGN_GUI_PLUGIN_PATH
→ GZ_GUI_PLUGIN_PATH
```

### 7. SDF system plugin

上游 Jazzy 使用：

```xml
<plugin
    filename="gz-sim-physics-system.so"
    name="gz::sim::systems::Physics"/>

<plugin
    filename="gz-sim-scene-broadcaster-system.so"
    name="gz::sim::systems::SceneBroadcaster"/>

<plugin
    filename="gz-sim-user-commands-system.so"
    name="gz::sim::systems::UserCommands"/>

<plugin
    filename="gz-sim-sensors-system.so"
    name="gz::sim::systems::Sensors"/>

<plugin
    filename="gz-sim-imu-system.so"
    name="gz::sim::systems::Imu"/>
```

因此旧：

```xml
<plugin
    filename="libignition-gazebo-physics-system.so"
    name="ignition::gazebo::systems::Physics"/>
```

直接改为：

```xml
<plugin
    filename="gz-sim-physics-system.so"
    name="gz::sim::systems::Physics"/>
```

机器人模型中的：

```xml
ignition-gazebo-joint-state-publisher-system
ignition-gazebo-joint-controller-system
```

改为：

```xml
gz-sim-joint-state-publisher-system.so
gz-sim-joint-controller-system.so
```

自定义 plugin 保持 filename：

```xml
<plugin
    filename="MecanumDrive2"
    name="gz::sim::systems::MecanumDrive2"/>

<plugin
    filename="LightBarController"
    name="gz::sim::systems::LightBarController"/>

<plugin
    filename="ProjectileShooter"
    name="gz::sim::systems::ProjectileShooter"/>
```

### 8. Launch

当前：

```python
"gz_version": "6"
```

应改：

```python
"gz_version": "8"
```

或者直接移除，让 Jazzy 的 `ros_gz_sim` 使用默认 Harmonic。

### 9. Bridge YAML

把：

```yaml
gz_type_name: "ignition.msgs.Odometry"
```

统一改为：

```yaml
gz_type_name: "gz.msgs.Odometry"
```

其他类型同理。

`spawn_robots.launch.py` 中：

```python
"ign",
"service",
"ignition.msgs.StringMsg",
"ignition.msgs.Boolean",
```

改成：

```python
"gz",
"service",
"gz.msgs.StringMsg",
"gz.msgs.Boolean",
```

### 推荐迁移策略

| 类别 | 处理方式 |
|---|---|
| `rmoss_gz_plugins` | 以上游 Jazzy 为基线，merge 你们自己的修改 |
| `rmoss_gz_base` | 以上游 Jazzy 为基线 |
| `rmoss_gz_cam` | 以上游 Jazzy 为基线 |
| `rmoss_gz_bridge/pose_bridge` | 以上游 Jazzy 为基线 |
| `rmoss_gz_bridge/rfid_bridge` | 手动按 Jazzy API 迁移 |
| `gui_plugins` | 手动迁 `gz-gui8` |
| `rmu_gazebo_simulator` | 手动改 launch / bridge / world |
| `pb2025_robot_description` | 手动改 SDF plugin |
| `rmoss_gz_resources` | 手动改 SDF / env |

---

## 用户

执行 patch

---

## 助手

由于当前 GitHub 连接对目标仓库只有读权限，无法直接向远端分支提交，因此改为生成一个可直接在本地仓库执行的迁移 patch 脚本。

脚本目标：

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
gz-sim8
gz-msgs10
gz-transport13
C++17
ros_gz
```

脚本处理内容包括：

- `ignition::* → gz::*`
- `ignition/gazebo → gz/sim`
- `IGNITION_ADD_PLUGIN → GZ_ADD_PLUGIN`
- `IGNITION_ADD_PLUGIN_ALIAS → GZ_ADD_PLUGIN_ALIAS`
- `ignerr / ignmsg → gzerr / gzmsg`
- Fortress CMake 依赖迁移到 Harmonic
- `C++14 → C++17`
- `IGN_GAZEBO_RESOURCE_PATH → GZ_SIM_RESOURCE_PATH`
- `IGN_GAZEBO_SYSTEM_PLUGIN_PATH → GZ_SIM_SYSTEM_PLUGIN_PATH`
- `IGN_GUI_PLUGIN_PATH → GZ_GUI_PLUGIN_PATH`
- `ignition.msgs.* → gz.msgs.*`
- SDF built-in system plugin filename 迁移
- `gz_version 6 → 8`
- `ign service → gz service`
- `<ignition-gui> → <gz-gui>`
- `resource/ign → resource/gz`
- 保留项目自定义的 `rfid_bridge`

生成文件：

```text
apply_jazzy_harmonic_patch.py
JAZZY_HARMONIC_PATCH_README.md
```

运行方式：

```bash
cd gazebo_simulator_origin

python3 apply_jazzy_harmonic_patch.py . --dry-run --show-diff

python3 apply_jazzy_harmonic_patch.py .
```

然后执行：

```bash
rm -rf build install log

source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --rosdistro jazzy

colcon build --symlink-install
```

patch 脚本已经进行了：

```text
py_compile: OK
smoke test: PASS
```

验证了包括：

- `ignition/gazebo/System.hh → gz/sim/System.hh`
- `ignition/transport/Node.hh → gz/transport/Node.hh`
- `namespace ignition → namespace gz`
- `namespace gazebo → namespace sim`
- `GZ_ADD_PLUGIN`
- `gz_version = 8`
- `GZ_SIM_RESOURCE_PATH`
- `<gz-gui>`

等核心迁移规则。

---

## 当前结论

这个项目从 Humble / Fortress 迁移到 Jazzy / Harmonic，最合适的方式不是完全手工从零改，而是：

1. 以 `robomaster-oss/rmoss_gazebo:jazzy` 作为 `rmoss_gz_*` 模块的基准。
2. 对 ECUST 项目自己的扩展，例如 `rfid_bridge`、GUI、world、PB2025 robot description 做定向迁移。
3. 第一阶段目标先做到：
   - 编译通过；
   - Gazebo world 能加载；
   - 自定义 system plugin 能被加载；
   - ROS ↔ Gazebo bridge 正常。
4. 再根据第一次 `colcon build` 和 Gazebo runtime 报错处理 Harmonic API 中不能机械替换的少量兼容问题。

最终目标栈：

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
Gazebo Sim 8
gz-msgs10
gz-transport13
gz-gui8
C++17
ros_gz
```
