# Ubuntu 迁移与验证

## 1. 迁移

Windows 工作区：

~~~text
H:\Desktop\rosstudy\dev_ws_sdf_gripper
~~~

把整个目录复制到 Ubuntu，目标建议为：

~~~text
~/Desktop/dev_ws_sdf_gripper
~~~

本目录只包含源码和文档，不包含 build、install、log。不要复制 Windows 构建产物。

若 ROS 2 输出 `getcwd failed No such file or directory`，先执行 `cd ~/Desktop/dev_ws_sdf_gripper`。这表示当前终端停留在已被删除或移动的目录；它不是控制器报错。

## 2. 依赖

~~~bash
sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-dev \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro \
  ros-humble-tf2-geometry-msgs \
  ros-humble-joint-state-publisher-gui
~~~

## 3. 编译

~~~bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install \
  --packages-select \
  cr7as_grasp \
  cr7as_grasp_moveit_config \
  cr7as_vision_grasp \
  cr7as_gazebo_grasp \
  cr7as_vision_motion \
  dobot_cr7_description
source install/setup.bash
~~~

## 4. Xacro 检查

~~~bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 run xacro xacro \
  ~/Desktop/dev_ws_sdf_gripper/src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro \
  use_sdf_closure:=true \
  sdf_left_pose_xyz="0.022 0 0" \
  sdf_right_pose_xyz="0.022 0 0" \
  sdf_left_axis="0 0 1" \
  sdf_right_axis="0 0 1" \
  > /tmp/cr7as_ag165.urdf
check_urdf /tmp/cr7as_ag165.urdf
~~~

输出中必须有 ag165_joint1_left、ag165_link2_left、ag165_link3_left、ag165_joint1_right、ag165_link2_right、ag165_link3_right 和 grasp_tcp。

## 5. 分阶段启动

### 5.1 CR7AS + AG165

~~~bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py
~~~

另开终端检查：

~~~bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states
~~~

首次测试先给较小的目标，观察实际位置：

~~~bash
cd ~/Desktop/dev_ws_sdf_gripper
ros2 action send_goal --feedback \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [ag165_joint1_left], points: [{positions: [0.20], time_from_start: {sec: 5, nanosec: 0}}]}}"
~~~

同时在另一终端执行：

~~~bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joint_states
~~~

在 `name` 数组中找到 `ag165_joint1_left` 的索引，读取 `position` 数组中的同一索引；不能把 `position` 第一项直接当作夹爪位置。action feedback 的 `actual.positions` 应随着 `desired.positions` 改变。成功后再试 0.62 rad，闭合目标为 0.05 rad。若 Gazebo 抖动或连杆分离，立即停止动作并保留日志。

### 5.2 完整视觉流程

~~~bash
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
ros2 node list | grep -E 'gazebo|move_group|pick_place|attachment|rgb|pixel'
ros2 service list | grep -E 'gazebo_grasp|attach|detach|get_entity_state|set_entity_state'
ros2 topic echo /vision_guided_motion/pick_place_status
ros2 topic echo /gazebo_grasp/attachment_status
ros2 service call /vision_guided_motion/execute_pick_place std_srvs/srv/Trigger "{}"
~~~

### 5.3 附着服务

~~~bash
ros2 service type /gazebo_grasp/attach
ros2 service type /gazebo_grasp/detach
ros2 service type /get_entity_state
ros2 service type /set_entity_state
ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: red_test_box, reference_frame: world}"
~~~

服务名存在不代表服务器节点仍活着；还要检查 attachment follower 节点和日志。

## 6. 重启原则

修改 Xacro、YAML 或 launch 后，重新 colcon build 并 source install/setup.bash，然后完全重启 Gazebo、控制器和 MoveIt。不要同时运行旧 AG95 入口、AG165 入口和第二个 robot_state_publisher。

## 7. AG165 闭环转换与插件验收（必须执行）

本工作区从 `ag165_90_4_full_gripper.sdf` 重建了 `ag165_macro.xacro` 的 joint 坐标。不要仅凭 STL 是否显示判断闭环是否生效；必须检查 URDF 转换后的最终 SDF：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash

ros2 run xacro xacro \
  ~/Desktop/dev_ws_sdf_gripper/src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro \
  use_sdf_closure:=true > /tmp/cr7as_ag165.urdf

gz sdf -p /tmp/cr7as_ag165.urdf > /tmp/cr7as_ag165_converted.sdf

grep -n -E \
'ag165_left_link2_to_link3_sdf|ag165_right_link2_to_link3_sdf' \
/tmp/cr7as_ag165_converted.sdf

ls ~/Desktop/dev_ws_sdf_gripper/install/cr7as_grasp/lib/libag165_gripper_mimic_plugin.so
grep -n 'ag165_gripper_mimic' /tmp/cr7as_ag165_converted.sdf
```

第一个 `grep` 必须输出左右两条 pin 闭环；`ls` 必须找到插件；最后一个 `grep` 必须找到 right-joint follower plugin。三者任一缺失时，禁止继续进行抓取测试。

启动后再验证被动关节物理状态真实发布：

```bash
ros2 topic echo /joint_states | grep -E \
'ag165_joint1_left|ag165_joint2_left|ag165_joint3_left|ag165_joint1_right|ag165_joint2_right|ag165_joint3_right'
```

预期：仅 `ag165_joint1_left` 接收 trajectory command；其余五个关节有 position / velocity 状态，但没有第二个夹爪 controller。


启动 `gazebo_ag165_control.launch.py` 后还应检查 plugin 节点服务：

```bash
ros2 service list | grep mimic_enable
```

预期至少出现：

```text
/cr7as_ag165/mimic_enable
```

如果不存在，优先检查 Gazebo 输出中是否有 `Failed to load plugin libag165_gripper_mimic_plugin.so`，以及 `GAZEBO_PLUGIN_PATH` 是否包含 `install/cr7as_grasp/lib`。

## 8. Action 成功但 AG165 实体不动

AG165 是一侧主动、右侧 PD follower、四个被动关节加两条 SDF pin 的四连杆。`gazebo_ros2_control` 不能保持未被 trajectory controller 认领的被动关节，否则闭环被锁死。当前整机文件必须包含：

```xml
<hold_joints>false</hold_joints>
```

修改该值后必须完全重启 Gazebo，不能只重启 controller。先确认安装后的 Xacro 已同步：

```bash
grep -n -A4 -B2 'hold_joints' \
  ~/Desktop/dev_ws_sdf_gripper/install/cr7as_grasp/share/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro
```

期望看到 `false`。若仍是 `true`，说明 Windows 修改的源码尚未同步，或 Ubuntu 未重新 `colcon build`。

2026-10-12 已确认 `ag165_joint1_left/position [claimed]`，同时 `/cr7as_ag165/mimic_enable` 存在；原 action feedback 却持续显示 `desired≈0.20 rad`、`actual≈-0.007 rad`。两个接口存在只能证明控制器与右侧跟随插件已加载，不能证明物理主关节被驱动。本版在 `cr7as_ag165_gazebo.urdf.xacro` 给左主动关节添加 `pos_kp=2.5`、`pos_ki=0`、`pos_kd=0.35`；GazeboSystem 由此把 position 命令转换为 `SetForce` 的 PID 力矩。`ag165_gazebo_controllers.yaml` 还设置了终点误差 0.03 rad 和 5 秒附加到位时间，防止不动仍 `SUCCEEDED`。

Ubuntu 重新编译并彻底重启 Gazebo 后验证安装文件：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
grep -n -A9 'joint name="ag165_joint1_left"' install/cr7as_grasp/share/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro | tail -n 12
grep -n -A15 '^gripper_controller:' install/cr7as_grasp/share/cr7as_grasp/config/ag165_gazebo_controllers.yaml
ros2 control list_hardware_interfaces -c /controller_manager | grep -A2 -B2 ag165_joint1_left
```

Gazebo 启动日志应出现类似 `Found position PIDs for joint: ag165_joint1_left`。若没有，先检查运行的是否为此工作区的新模型。若有 PID 日志但 0.20 rad 目标时 actual 仍不变，请保存 Gazebo 启动日志、action feedback 和 `/joint_states`；再检查最终 SDF 闭环关节、接触干涉和 PID 力矩，而不是再次盲目增大增益。
