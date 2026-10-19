# SW2URDF 导出包转 ROS 2 Humble：`cr7as_grasp` 实例

本文说明如何把 SolidWorks SW2URDF 导出的 ROS1/Catkin 包转换为 ROS 2 Humble，并解释本实例中 Gazebo 报错的原因。

## 1. 原始包和目标包的差异

| 项目 | SW2URDF 原始输出 | ROS 2 Humble 目标 |
| --- | --- | --- |
| 构建系统 | `catkin` | `ament_cmake` |
| 启动文件 | `display.launch`、`gazebo.launch` XML | `display.launch.py`、`gazebo_control.launch.py` Python |
| 可视化 | `rviz` | `rviz2` |
| Gazebo 生成 | ROS1 `spawn_model -file` | ROS2 `spawn_entity.py -topic robot_description` |
| 控制接口 | 没有 `ros2_control` | `gazebo_ros2_control` + 控制器 YAML |
| mesh URI | `package://cr7as_grasp/...` | 保持为 `package://cr7as_grasp/...`，但由 ament 安装并解析 |
| 根坐标系 | `b0`（带惯性） | `world -> base_link -> b0` |

URDF 的 link/joint XML 本身不是“ROS1 专用格式”；主要转换是外层功能包、launch、Gazebo 插件和资源安装方式。

## 2. 本次 Gazebo 报错的直接原因

错误是：

```text
ValueError: Unicode strings with encoding declaration are not supported.
```

原文件第一行是：

```xml
<?xml version="1.0" encoding="utf-8"?>
```

ROS2 launch 用 `read_text()` 得到 Python Unicode 字符串，再通过 `robot_description` 话题交给 `gazebo_ros/spawn_entity.py`。该节点使用 lxml 的字符串解析接口；字符串已经是 Unicode 时，不能再包含 XML 的 `encoding` 声明。

修改为删除声明，或至少删除 `encoding="utf-8"`：

```xml
<!-- 删除这一行 -->

<robot name="cr7as_grasp">
```

本包的 `cr7as_grasp.urdf` 和 `cr7as_grasp_gazebo.urdf` 均已删除该声明。不要把这个问题误判为关节坐标、质量或 STL 几何错误。

本实例还发现 `meshes/grasp_tcp.STL` 只有 84 字节且没有三角面。它对应的是工具坐标 link，不能作为可渲染网格；因此最终 URDF 保留 `grasp_tcp` 的 inertial 和固定 TF，但删除其空的 `visual` / `collision`。如果需要显示工具外形，应从 CAD 重新导出一个有三角面的 STL，再恢复这两个元素。

## 3. 为什么增加 `world` 和 `base_link`

原始根节点是带惯性的 `b0`，Robot State Publisher 会警告：

```text
The root link b0 has an inertia specified in the URDF, but KDL does not support a root link with an inertia.
```

这通常不是 Gazebo 生成失败的原因，但应按标准做法增加无惯性的固定根：

```xml
<link name="world" />
<joint name="world_to_base_link" type="fixed">
  <parent link="world" />
  <child link="base_link" />
</joint>
<link name="base_link" />
<joint name="base_link_to_b0" type="fixed">
  <parent link="base_link" />
  <child link="b0" />
</joint>
```

RViz2 版可以省略 `world`，但保留 `base_link -> b0`；Gazebo 版保留两级根，固定机器人在世界坐标中生成。

## 4. Gazebo 版必须增加的内容

仅有 SolidWorks URDF 不能驱动 Gazebo 关节。仿真 URDF 需要：

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <!-- j1 到 j6 的 position command/state interface -->
</ros2_control>

<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find cr7as_grasp)/config/gazebo_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

因为这里使用的是普通 URDF，不是 xacro，launch 文件会把 `$(find cr7as_grasp)/config/gazebo_controllers.yaml` 替换为安装后的绝对路径。

## 5. 可复用检查顺序

1. 用 XML 解析器打开 URDF，确认只有一个根机器人元素；
2. 检查 `package://包名/meshes/文件名` 与实际大小写完全一致；
3. 检查所有 parent/child link 都存在，且整棵树只有一个根；
4. ROS2 launch 读取的 URDF 不要包含 `encoding` XML 声明；
5. RViz2 先运行 `display.launch.py`，确认 `RobotModel` 和 TF；
6. Gazebo 再运行 `gazebo_control.launch.py`，确认实体生成后再检查控制器；
7. `j1` 到 `j6` 必须在 URDF、`ros2_control`、控制器 YAML 和轨迹消息中逐字一致；
8. 最后才校准关节数据、碰撞模型和实机安全参数。

## 6. 仍需人工确认的内容

本转换修复的是 ROS2/Humble 的包格式和启动链路，不会自动判断 SolidWorks 中的机构设计是否正确。以下内容仍需依据 CAD、厂家资料或实测确认：关节原点和轴向、真实限位、力矩/速度、惯量、碰撞几何、末端工具坐标系以及实机控制器参数。

## 7. `/controller_manager` 不存在：本项目实际遇到的错误

当 Gazebo 日志出现：

```text
SpawnEntity: Successfully spawned entity [cr7as_grasp]
[gazebo_ros2_control]: Loading gazebo_ros2_control plugin
parser error Couldn't parse parameter override rule:
'--param robot_description:=<!-- This URDF ...'
```

而控制器 spawner 一直显示：

```text
waiting for service /controller_manager/list_controllers to become available
```

故障链是：机器人生成成功，但 `gazebo_ros2_control` 在解析 `robot_description` 时失败，所以没有创建 `/controller_manager`；随后控制器无法加载，机械臂会在重力下塌落。这个现象不是关节坐标错误。

本项目的修复是：

1. 删除 Gazebo URDF 的 XML 声明 `<?xml ... encoding="utf-8"?>`；
2. 删除 Gazebo URDF 开头的 SolidWorks 多行 `<!-- ... -->` 注释以及其他 XML 注释；
3. 保留 `world -> base_link -> b0`、`ros2_control`、`libgazebo_ros2_control.so` 和控制器 YAML；
4. 重新安装包并确认安装后的文件已更新，而不是只修改 `src` 文件。

Ubuntu 中的检查和重建顺序：

```bash
cd ~/Desktop/dev_ws
source /opt/ros/humble/setup.bash

grep -n "<!--\|encoding=" \
  src/cr7as_grasp/urdf/cr7as_grasp_gazebo.urdf

rm -rf build/cr7as_grasp install/cr7as_grasp
colcon build --symlink-install --packages-select cr7as_grasp
source install/setup.bash

grep -n "<!--\|encoding=" \
  install/cr7as_grasp/share/cr7as_grasp/urdf/cr7as_grasp_gazebo.urdf
```

两个 `grep` 都不应有输出。然后再启动：

```bash
ros2 launch cr7as_grasp gazebo_control.launch.py
```

必须看到 Gazebo 进程已经启动并且机器人生成后，再在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager
```

如果仍然没有服务，优先检查 Gazebo 终端中的 `gazebo_ros2_control` 错误，以及插件是否安装：

```bash
ros2 pkg prefix gazebo_ros2_control
grep -n "ros2_control\|libgazebo_ros2_control" \
  install/cr7as_grasp/share/cr7as_grasp/urdf/cr7as_grasp_gazebo.urdf
```

只有 `/controller_manager` 存在后，`joint_state_broadcaster` 和 `arm_controller` 才有可能变为 `active`。

## 8. URDF 首字符和“源文件已改、安装文件未改”

URDF 文件的第一个有效字符必须是 `<`（通常是 `<robot`）。不要在文件开头留下中文字符、复制提示符、Markdown 围栏或其他文本，例如：

```text
三
<robot name="cr7as_grasp">
```

这类多余字符会让 XML 解析失败，或让 Gazebo 插件把整段内容识别成非法参数。可用下面命令检查首行：

```bash
head -n 3 src/cr7as_grasp/urdf/cr7as_grasp_gazebo.urdf
```

第一行应为空行或 `<robot`，不能出现中文或其他提示文本。修改 `src` 后，必须重新构建并重新 `source`，因为 ROS 2 实际读取的是 `install` 中的副本：

```bash
cd ~/Desktop/dev_ws
source /opt/ros/humble/setup.bash
rm -rf build/cr7as_grasp install/cr7as_grasp
colcon build --symlink-install --packages-select cr7as_grasp
source install/setup.bash

head -n 3 install/cr7as_grasp/share/cr7as_grasp/urdf/cr7as_grasp_gazebo.urdf
```

`src` 与 `install` 中的首行和 XML 内容必须一致。若 `ros2 launch` 报错，但 `src` 文件看起来正确，优先检查 `install` 文件是否仍是旧版本。
