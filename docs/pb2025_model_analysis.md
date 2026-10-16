# PB2025 哨兵模型：SDF、关节与碰撞分析

分析对象是本仓库实际启动的 `pb2025_sentry_robot.sdf.xmacro` 展开结果，已移除工业相机。以下连接图表示 parent/child 拓扑；位姿仍需遵循 SDF 的 `relative_to` 和默认坐标语义。未启动动力学仿真，配置疑点不等同于已经复现的故障。

## 1. 文件组成与加载路径

- `src/rmu_gazebo_simulator/launch/spawn_robots.launch.py` 使用 xmacro 展开机器人 SDF，通过 Gazebo create 生成机器人，再用 `sdformat_tools` 转为 URDF 供 robot_state_publisher 发布 TF。
- `src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro`：机器人总装入口，组合本体、Mid360 和控制插件。
- `src/rmoss_gz_resources/resource/models/rmua19_standard_robot/rmua19_standard_robot.def.xmacro`：底盘、云台、装甲、灯条及测速模块的装配关系。
- 同目录 `chassis_wheel.def.xmacro`：左右轮的关节、惯性、显示和碰撞。
- `src/pb2025_robot_description/resource/models/mid360/model.sdf.xmacro`：雷达固定关系、显示模型、IMU 与 GPU 激光雷达。
- 装甲、灯条、测速模块由 `rm21_*` 对应的 `.def.xmacro` 引入。总装实际引用的是宏定义，不是目录里独立演示用的 `model.sdf`。
- 外观主要使用 DAE 网格，碰撞使用简化盒体和圆柱；它是在 RMUA 2019 标准步兵基础上修改组合的模型。

## 2. 展开结果与连接树

移除相机后：**19 个 link，18 个 joint，其中 6 个 revolute、12 个 fixed；25 个 collision（21 个 box、4 个 cylinder）；4 个 sensor、6 个模型级插件实例。** 原相机占 2 个 link、2 个 fixed joint、1 个碰撞盒；实际总装宏中相机 sensor 此前已不存在。

```text
aft_mapped
├── base_link                         [aft_to_base_link, fixed]
│   ├── chassis                       [gimbal_yaw_joint, revolute Z]
│   │   ├── front_left_wheel          [revolute Y]
│   │   ├── front_right_wheel         [revolute Y]
│   │   ├── rear_left_wheel           [revolute Y]
│   │   ├── rear_right_wheel          [revolute Y]
│   │   ├── armor_support_frame_0 → armor_0  [fixed → fixed]
│   │   ├── armor_support_frame_1 → armor_1  [fixed → fixed]
│   │   ├── armor_support_frame_2 → armor_2  [fixed → fixed]
│   │   ├── armor_support_frame_3 → armor_3  [fixed → fixed]
│   │   └── light_indicator           [fixed]
│   └── gimbal_pitch                  [gimbal_pitch_joint, revolute Y]
│       └── speed_monitor             [fixed]
└── livox_frame                       [livox_joint, fixed]
```

`aft_mapped` 是 parent/child 树根；SDF 中首先声明的 link 则是 `base_link`。不要仅凭名字把 `base_link` 当作物理底盘，实际轮子与底盘碰撞都属于 `chassis`。

## 3. 所有关节

位姿单位：米、弧度。标注“默认”表示原文件没有写 `relative_to`；不能直接把这种 SDF joint pose 当作 URDF 的 parent→child origin。

| joint | 类型 | parent → child | pose | pose 参考系 | axis | 限位 |
|---|---|---|---|---|---|---|
| `aft_to_base_link` | fixed | `aft_mapped` → `base_link` | `0 0 0 0 0 1.09` | 默认 | — | — |
| `front_left_wheel_joint` | revolute | `chassis` → `front_left_wheel` | `0.207 0.194 0 0 0 0` | chassis | 0 1 0 | [-1.79769e+308, 1.79769e+308] |
| `front_right_wheel_joint` | revolute | `chassis` → `front_right_wheel` | `0.207 -0.194 0 0 0 0` | chassis | 0 1 0 | [-1.79769e+308, 1.79769e+308] |
| `rear_left_wheel_joint` | revolute | `chassis` → `rear_left_wheel` | `-0.207 0.194 0 0 0 0` | chassis | 0 1 0 | [-1.79769e+308, 1.79769e+308] |
| `rear_right_wheel_joint` | revolute | `chassis` → `rear_right_wheel` | `-0.207 -0.194 0 0 0 0` | chassis | 0 1 0 | [-1.79769e+308, 1.79769e+308] |
| `gimbal_yaw_joint` | revolute | `base_link` → `chassis` | `0 0 -0.5 0 0 0` | base_link | 0 0 1 | [-1.79769e+308, 1.79769e+308] |
| `gimbal_pitch_joint` | revolute | `base_link` → `gimbal_pitch` | `0 0.004 -0.366 0 0 0` | base_link | 0 1 0 | [-0.785, 0.5652] |
| `armor_support_frame_0_joint` | fixed | `chassis` → `armor_support_frame_0` | `0.197 0 0.061 0 0 0` | chassis | — | — |
| `armor_0_joint` | fixed | `armor_support_frame_0` → `armor_0` | `0.075 0 -0.071 0 -0.262 0` | armor_support_frame_0 | — | — |
| `armor_support_frame_1_joint` | fixed | `chassis` → `armor_support_frame_1` | `0 0.116 0.061 0 0 1.5708` | chassis | — | — |
| `armor_1_joint` | fixed | `armor_support_frame_1` → `armor_1` | `0.075 0 -0.071 0 -0.262 0` | armor_support_frame_1 | — | — |
| `armor_support_frame_2_joint` | fixed | `chassis` → `armor_support_frame_2` | `-0.197 0 0.061 0 0 3.14` | chassis | — | — |
| `armor_2_joint` | fixed | `armor_support_frame_2` → `armor_2` | `0.075 0 -0.071 0 -0.262 0` | armor_support_frame_2 | — | — |
| `armor_support_frame_3_joint` | fixed | `chassis` → `armor_support_frame_3` | `0 -0.116 0.061 0 0 -1.5708` | chassis | — | — |
| `armor_3_joint` | fixed | `armor_support_frame_3` → `armor_3` | `0.075 0 -0.071 0 -0.262 0` | armor_support_frame_3 | — | — |
| `light_indicator_joint` | fixed | `chassis` → `light_indicator` | `-0.207 0 0.1 0 0 0` | chassis | — | — |
| `speed_monitor_joint` | fixed | `gimbal_pitch` → `speed_monitor` | `0.07 0 0 0 0 0` | gimbal_pitch | — | — |
| `livox_joint` | fixed | `aft_mapped` → `livox_frame` | `-0.2 0 -0.33 0.0 0.0 0.0` | aft_mapped | — | — |

- 四轮中心相对 chassis 为 `(±0.207, ±0.194, 0)`，轴距 0.414 m、轮距 0.388 m；半径 0.0758 m、宽度 0.0452 m。轮轴是局部 Y。
- Pitch 范围为 −0.785～0.5652 rad，约 −45°～32.38°。Yaw 和四轮使用极大的上下限，配置意图为不限制转角。
- `gimbal_yaw_joint` 是 `base_link → chassis`；它直接连接并驱动底盘侧相对 base_link 的旋转。若按常见“底盘固定、云台 yaw 在上”的直觉理解，会读反该模型。
- `aft_to_base_link` 的 joint pose 为 `0 0 0 0 0 1.09`，未显式指定 relative_to。SDF 中 joint pose 默认相对子 link；单独这个数不能证明两个 link 的相对 yaw 就是 1.09。应结合 link pose 解析实际帧关系。
- Mid360 joint 挂到 `aft_mapped`，位置 `(-0.2, 0, -0.33)`；`livox_frame` 又相对该 joint 设置 yaw = −1.095。它不是直接挂在 chassis 上，底盘相对 yaw 转动时需要注意雷达所依附的刚体。

## 4. 碰撞体清单

下表的 pose 为碰撞体在所属 link 下的局部位姿；盒体尺寸为 XYZ 全长，圆柱参数为半径和长度。

| link | collision | 几何与尺寸（m） | 局部 pose |
|---|---|---|---|
| `chassis` | `base_link_base_collision` | box 0.36 0.24 0.13 | `0 0 0.045 0 0 0` |
| `chassis` | `base_link_projectile_container_collision` | box 0.1 0.18 0.146 | `-0.122 0 0.177 0 0 0` |
| `chassis` | `base_link_fornt_wheel_holder_collision` | box 0.055 0.367 0.055 | `0.208 0 0 0 0 0` |
| `chassis` | `base_link_rear_wheel_holder_collision` | box 0.055 0.367 0.055 | `-0.208 0 0 0 0 0` |
| `front_left_wheel` | `front_left_wheel_collision` | cylinder r=0.0758 l=0.0452 | `0 0 0 1.5707 0 0` |
| `front_right_wheel` | `front_right_wheel_collision` | cylinder r=0.0758 l=0.0452 | `0 0 0 1.5707 0 0` |
| `rear_left_wheel` | `rear_left_wheel_collision` | cylinder r=0.0758 l=0.0452 | `0 0 0 1.5707 0 0` |
| `rear_right_wheel` | `rear_right_wheel_collision` | cylinder r=0.0758 l=0.0452 | `0 0 0 1.5707 0 0` |
| `aft_mapped` | `collision` | box 0.065 0.117 0.177 | `0 0 1.0885 0 0 0` |
| `gimbal_pitch` | `collision_main` | box 0.1 0.0915 0.08 | `0.015 0 0 0 0 0` |
| `gimbal_pitch` | `collision_box` | box 0.14801 0.07 0.065 | `-0.011 0 0.043 0 0 0` |
| `armor_support_frame_0` | `collision` | box 0.03 0.11 0.08 | `0.015 0 0 0 0 0` |
| `armor_0` | `collision` | box 0.013 0.135 0.125 | `0 0 0.0625 0 0 0` |
| `armor_0` | `target_collision` | box 0.008 0.135 0.125 | `0.012 0 0.0625 0 0 0` |
| `armor_support_frame_1` | `collision` | box 0.03 0.11 0.08 | `0.015 0 0 0 0 0` |
| `armor_1` | `collision` | box 0.013 0.135 0.125 | `0 0 0.0625 0 0 0` |
| `armor_1` | `target_collision` | box 0.008 0.135 0.125 | `0.012 0 0.0625 0 0 0` |
| `armor_support_frame_2` | `collision` | box 0.03 0.11 0.08 | `0.015 0 0 0 0 0` |
| `armor_2` | `collision` | box 0.013 0.135 0.125 | `0 0 0.0625 0 0 0` |
| `armor_2` | `target_collision` | box 0.008 0.135 0.125 | `0.012 0 0.0625 0 0 0` |
| `armor_support_frame_3` | `collision` | box 0.03 0.11 0.08 | `0.015 0 0 0 0 0` |
| `armor_3` | `collision` | box 0.013 0.135 0.125 | `0 0 0.0625 0 0 0` |
| `armor_3` | `target_collision` | box 0.008 0.135 0.125 | `0.012 0 0.0625 0 0 0` |
| `light_indicator` | `light_bar_collision` | box 0.05 0.28 0.04 | `0 0 0.06 0 0 0` |
| `speed_monitor` | `collision` | box 0.11 0.043 0.041 | `0.055 0 0 0 0 0` |

`base_link` 和 `livox_frame` 没有碰撞体：前者主要用于坐标关系；后者有显示网格和传感器，但雷达外壳本身不参与物理接触。碰撞简化意味着模型可能在视觉外壳接触前/后才发生物理接触。

## 5. 传感器与控制

| 模块 | 当前配置 |
|---|---|
| 底盘 IMU | `chassis/base_link_imu`，200 Hz |
| 云台 IMU | `gimbal_pitch/gimbal_imu`，200 Hz |
| Mid360 IMU | `livox_frame/mid360_imu`，200 Hz；桥接 `/livox/imu` |
| Mid360 激光 | GPU lidar，100 Hz，875 水平采样 × 32 垂直线，水平约 360°、垂直 −7°～52°，距离 0.1～40 m；桥接原始 PointCloud2 后经转换节点处理 |
| RPLIDAR A2 | 总装挂载已注释，模型资源仍在；当前不生成该传感器 |
| 工业相机 | 已删除模型资源、总装引用、独立图像驱动 `rmoss_gz_cam` 和对应迁移脚本模板 |

模型级插件：JointStatePublisher 发布关节状态；LightBarController 管理四块装甲、状态灯和测速模块灯条；MecanumDrive2 操作四轮并使用 chassis；两个 JointController 分别控制 yaw/pitch，`use_force_commands=false`；ProjectileShooter 从 speed_monitor 发射 17 mm 弹丸，初速 20 m/s。插件之外，`rmoss_gz_base` 负责 ROS 控制接口，`ros_gz_bridge` 负责已配置的状态、点云和 IMU 桥接。

## 6. 需要优先核对的配置

1. **aft_mapped 显示与碰撞错位风险**：visual pose z = −0.4，而碰撞盒中心 z = 1.0885；两者局部参考原点相差 1.4885 m。碰撞盒 z 范围约为 [1.0, 1.177] m。DAE 网格内部原点也会影响显示位置，因此不能仅凭 pose 差值断言最终表面错位，但很值得在 Gazebo 中开启碰撞显示核对，尤其是钻桥/顶部接触行为。
2. **底盘质心偏移风险**：chassis 惯性原点 z = −0.6，而主碰撞盒中心 z = 0.045、轮中心 z = 0。质心在轮轴下方 0.6 m，是明显需要核对的物理参数，会影响稳定性和坡面响应。
3. **SDF 和 URDF 坐标语义差异风险**：当前 `sdf2urdf.py` 主要把 pose 数值转换为 origin，未解析 SDF `relative_to` 帧图。宏展开和转换成功不能证明 RViz TF 与 Gazebo 实体帧完全一致，尤其是 `aft_to_base_link` 的默认参考系，以及 livox link 自身的 −1.095 yaw。
4. **轮接触是圆柱近似**：每轮只有一个圆柱 collision，摩擦 `mu=mu2=0.2`；没有麦轮滚子的独立几何/关节。全向运动效果来自 MecanumDrive2 等控制逻辑及简化接触，不是逐滚子接触仿真。
5. **雷达没有碰撞壳体**：若关心窄门、桥下通过性，需要把雷达实物占用空间纳入碰撞模型；当前仅显示和采样。
6. **质量与 link 默认值**：显式声明质量合计 19.105 kg，base_link 没有惯性声明；这不是对物理引擎最终总质量的测量。应以 Gazebo 解析与固定关节合并后的物理属性确认。

以上结构与物理配置仅分析，除相机移除外未做自动修正。

## 7. 验证范围

- 使用实际启动使用的 `xmacro.xmacro4sdf.XMLMacro4sdf` 成功展开，并完成 SDF→URDF 转换。
- 展开结果不包含 industrial_camera link/joint/sensor；保留四个非相机传感器。
- Gazebo `gz sdf -k` 校验通过。
- `pb2025_robot_description`、`rmu_gazebo_simulator` 编译通过；这两个包当前没有注册自动测试。
- 未启动 Gazebo 做运动、碰撞和 TF 对齐的动态验证。
