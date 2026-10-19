> **当前交付覆盖说明（2026-10-12）**：本包使用 `cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro`，MoveIt 夹爪组只控制 `ag165_joint1_left`，末端为 `grasp_tcp`。当前 Gazebo 入口是 `cr7as_grasp gazebo_ag165_control.launch.py`，不是旧 AG95 的 `gazebo_control.launch.py`。迁移目录是 `~/Desktop/dev_ws_sdf_gripper`；以工作区根目录 README 和 docs 为准。
# 当前交付说明

2026-10-13：修复 `config/cr7as_grasp.srdf` 在 AG165 迁移时漏掉的六组机械臂相邻碰撞排除（`b0-l1` 至 `l5-l6`）。用户日志中的 `l1/l2` 起始碰撞导致四个预抓取候选全部失败；恢复项均来自原 CR7AS SRDF，并与当前 URDF 六个转动关节的父子 link 对应。非相邻连杆、桌面、工件和料箱仍保留碰撞检查。同步后在 Ubuntu 运行 `colcon build --symlink-install --packages-select cr7as_grasp_moveit_config`，source 并完全重启完整场景，再调用接触测试。Windows 仅完成 XML 与拓扑核对，未执行 MoveIt 实测。

> 本交付工作区的 MoveIt 模型已切换为 CR7AS + AG165-90；夹爪规划组只控制 ag165_joint1_left，其余 AG165 关节为 passive/mimic。完整验证入口和迁移命令以工作区根目录 README 与 docs/ 为准。

# cr7as_grasp_moveit_config

> 2026-10-13 实测纠偏：AG165 左主动关节从约 0 rad 向 0.62 rad 运动是**闭合**，因此 `config/cr7as_grasp.srdf` 中的 `gripper/open` 已改为 0.0、`gripper/close` 已改为 0.62。执行器 `pick_place.yaml` 和 C++ 默认值同步；请与这两处一起复制、编译并完全重启，不要只更新其中一个包。

> 2026-10-13 AG165 规划修复（待 Ubuntu 验证）：新接触测试日志明确报告 `l6` 与 `ag165_base_link` 在规划起始状态自碰撞。两个 link 通过固定的 `gripper_base` 安装，现仅在 `config/cr7as_grasp.srdf` 中增加这一对安装处的排除；旧 AG95 SRDF 的批量 `Never` 条目不复制，环境与工件碰撞保持启用。同时 AG165 整机 Xacro 恢复原六轴限位。同步两个配置文件、重编译 `cr7as_grasp cr7as_grasp_moveit_config` 并完全重启；若接下来报其他碰撞或 IK 失败，再依据新日志处理。本 README 下方旧 `dev_ws`/AG95 命令是历史说明，不是当前 `dev_ws_sdf_gripper` 的启动入口；请用工作区根 README 的 `demo_pick_place.launch.py`。

这是针对 `cr7as_grasp` 的 ROS 2 Humble / MoveIt 2 配置包。

## 放置方式

本配置包当前放在：

`H:\Desktop\rosstudy\cr7as_grasp\cr7as_grasp_moveit_config`

复制到 Ubuntu 工作空间时，需要让下面两个 ROS 包都位于 `~/Desktop/dev_ws/src` 下：

- `cr7as_grasp`
- `cr7as_grasp_moveit_config`

如果保持当前嵌套目录，复制整个 `cr7as_grasp` 后，colcon 会递归发现里面的 `cr7as_grasp_moveit_config/package.xml`；更推荐将 `cr7as_grasp_moveit_config` 单独复制为 `~/Desktop/dev_ws/src/cr7as_grasp_moveit_config`。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/dev_ws
colcon build --symlink-install --packages-select cr7as_grasp cr7as_grasp_moveit_config
source ~/Desktop/dev_ws/install/setup.bash
```

## 启动顺序：Gazebo + MoveIt

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws/install/setup.bash
ros2 launch cr7as_grasp gazebo_control.launch.py
```

终端 2：确认控制器：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws/install/setup.bash
ros2 launch cr7as_grasp_moveit_config move_group.launch.py
```

终端 4：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws/install/setup.bash
ros2 launch cr7as_grasp_moveit_config moveit_rviz.launch.py
```

也可以在不启动 Gazebo 的情况下，仅启动 MoveIt 节点和 RViz：

```bash
ros2 launch cr7as_grasp_moveit_config demo.launch.py
```

但进行 Gazebo 实际执行时，不要启动旧的 `urdf_ros2_moveit_config/demo.launch.py`，避免 FakeSystem 与 Gazebo 控制系统冲突。

