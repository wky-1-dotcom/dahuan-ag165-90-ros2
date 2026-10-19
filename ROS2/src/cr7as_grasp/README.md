# cr7as_grasp：CR7AS + AG165-90

> 2026-10-13 新增只读双侧 contact sensor，详见本文末尾“2026-10-13 双侧接触传感器（旁路诊断）”。Ubuntu 尚未验证生成的 Gazebo collision 名和话题。

本包提供 CR7AS 六轴机械臂和大寰 AG165-90 夹爪的 ROS 2 Humble / Gazebo Classic 模型。当前完整流程默认使用 AG165，并在 Gazebo 物理层启用两侧四连杆 SDF 闭环。

> 2026-10-13 Ubuntu 动作方向实测：`ag165_joint1_left` 启动约 0 rad，命令 0.62 rad 后实际约 0.6198 rad，用户确认是**闭合**。整机 Xacro 的初始值 0.0 不变；这就是近张开姿态。执行器与 MoveIt 的 open/close 参数已在各自包中纠偏，不需要也不应仅为“开机张开”把闭环单个主动关节初值改到 0.62。关节上限 0.93 不是验证过的闭爪目标。

> 2026-10-13 Gazebo 状态服务修复（待 Ubuntu 验证）：`launch/gazebo_ag165_control.launch.py` 曾启动通用默认 world，导致 `/get_entity_state` 不可用、预抓取成功后接触测试停止。现与旧启动方式一致，使用 `gzserver.launch.py` 加载已安装的 `worlds/empty_with_state.world`（含 `libgazebo_ros_state.so`），并单独包含 `gzclient.launch.py`。更新本包后重新编译并完全重启 Gazebo；在接触测试前用 `ros2 service list` 和一次实际 `GetEntityState` 调用核验。

> 2026-10-13 机械臂限位对照修复（待 Ubuntu 验证）：AG165 整机 Xacro 的 j1/j2/j4/j5/j6 曾被统一写成 `[-1.57,1.57]`，j3 也写成此范围；与旧 CR7AS 模型不一致。现恢复前五轴对应的原值：j1/j2/j4/j5/j6 `[-3.14,3.14]`，j3 `[-2.79,2.79]`。b0～l6 的 mesh、惯量、关节轴/原点、`grasp_tcp` 固定关节保持原样。新抓取模型同时需要 MoveIt SRDF 的固定安装碰撞豁免，详见工作区根 README。复制后重编译本包和 MoveIt 配置包并完全重启。

> 2026-10-13 虚拟相机视场调整：`urdf/cr7as_ag165_gazebo.urdf.xacro` 中相机位置改为 `base_link` 下 `(0, 0.80, 1.60)` m，水平视场改为 `1.3963` rad（约 80°）；图像仍为 `1280×720`。桌面 `z=0.62` m 处理论覆盖约 `1.64×0.93` m，须在 Ubuntu 完整场景中核对实际视野与机械臂遮挡。只需重新编译本包、source、完全重启 Gazebo；不改 `plane_z_m` 或夹爪控制器。

## 当前生效文件

- urdf/cr7as_ag165_gazebo.urdf.xacro：完整 Gazebo robot description；
- urdf/ag165_macro.xacro：AG165 link、碰撞、摩擦、右侧 PD 跟随和 Gazebo pin 闭环；
- launch/gazebo_ag165_control.launch.py：Gazebo、机器人、ros2_control 和三个控制器；
- config/ag165_gazebo_controllers.yaml：AG165 控制器参数；
- models/ag165_90_4/model.sdf：独立参考 SDF，不与完整 Xacro 同时 spawn；
- meshes/ag165/：AG165 网格。

## 控制器契约

~~~text
arm_controller    : j1 ... j6
gripper_controller: ag165_joint1_left
~~~

右侧主动镜像关节和两侧从动连杆不加入 ROS 2 控制器列表。右侧由 Gazebo mimic 插件以力矩跟随左侧，四个从动关节由 SDF 闭环约束求解。`GazeboArmSystem` 保留 j1～j6 原位置控制，`GazeboGripperSystem` 对左侧主动关节启用位置 PID；`ag165_gazebo_controllers.yaml` 配置真实位置误差容差。

## 启动

~~~bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py
~~~

夹爪 action 使用 ag165_joint1_left。完整视觉场景请从 cr7as_vision_motion 启动。不要同时运行旧 AG95 Gazebo launch、第二个 robot_state_publisher 或独立 AG165 SDF。

更详细的闭环、迁移和验收说明见工作区根目录 docs/。

## 2026-10-12 SDF 结构修正

`ag165_macro.xacro` 的 joint 原点、轴和限位以 `ag165_90_4_full_gripper.sdf` 为唯一真值。它安装 `libag165_gripper_mimic_plugin.so`，用力控方式驱动右侧主动连杆；四个被动连杆只发布物理状态。编译本包后必须确认：

```bash
ls ~/Desktop/dev_ws_sdf_gripper/install/cr7as_grasp/lib/libag165_gripper_mimic_plugin.so
```

如果 Gazebo 日志出现 `ag165_gripper_mimic_plugin: active joint not found` 或 follower joint not found，停止仿真并检查最终 SDF 中是否保留 `ag165_joint1_left` 和 `ag165_joint1_right`。

> 启动保持：完整模型使用 `<hold_joints>true</hold_joints>`，防止 arm_controller 接管前六轴下垂。Humble 的 GazeboSystem 只保持有命令接口的关节；五个 AG165 被动关节只声明 state interface，因此仍可自由满足 SDF 闭环。

2026-10-12 Ubuntu 反馈表明硬件接口已 `[claimed]`、mimic 插件已加载，但 `desired≈0.20 rad` 时 `actual≈-0.007 rad`。为左侧主动关节增加 `pos_kp=2.5`、`pos_kd=0.35`，令 GazeboSystem 通过 PID `SetForce` 驱动闭环。先测试 0.20 rad 的小动作，检查 `/joint_states` 和 action feedback，再测试完整张合。源工程的 `ag165_hold_plugin` 不能接收目标位置，因此未作为驱动器使用。

随后发现启动塌臂：单个 GazeboSystem 将夹爪 PID 模式应用到了同组件的机械臂 j1～j6，它们没有 PID 增益。现在拆为两个 `<ros2_control>` 硬件组件，不给六轴盲目添加示例中的大增益；同时恢复启动保持，整机 `spawn_entity` 的 z 恢复 0.0 m。j1～j6 本身已配置阻尼 `1.0`，五个 AG165 被动关节本身没有 command interface。

0.20 rad 小动作的画面显示连杆交叠。检查参考 SDF 后补齐左右闭环关节 `<pose>` 的 `relative_to="ag165_link2_left/right"`，使 `0.022 0 0` 按正确的 link2 局部坐标解释。原始 SDF 也使用 `self_collide=false`；若修正坐标后仍交叠，先采集关节反馈及最终 SDF，避免直接启用全部自碰撞。
# 2026-10-12 右侧跟随与抖动诊断

`src/ag165_gripper_mimic_plugin.cpp` 新增 `/<Gazebo模型名>/mimic_status` (`std_srvs/srv/Trigger`)。完整机器人默认模型名为 `cr7as_ag165`，即 `/cr7as_ag165/mimic_status`。Ubuntu 重新编译本包并重启 Gazebo 后调用：

```bash
ros2 service call /cr7as_ag165/mimic_status std_srvs/srv/Trigger '{}'
ros2 service call /cr7as_ag165/mimic_enable std_srvs/srv/SetBool '{data: true}'
```

确认 `active found=true`、`followers=1/1`、`enabled=true`，再发 0.20 rad action。若右侧不动，比较状态服务中的 `target`、`position`、`force`；服务存在本身不表示右关节被找到。不要在未定位右侧不动原因前，同时更改 pin 的 CFM/ERP 和左主动阻尼。当前物理参数仍按参考 SDF 保留。

Ubuntu 实测证明上一版 multiplier=-1 会使两指反向开合，虽然抽搐消失，但方向错误。当前默认 `ag165_soft_mimic_test:=true`：右侧 multiplier 恢复 +1，限位恢复 `[-0.05, 0.93]`，保留 offset=0、阻尼 0.15、PD `1.5/0.3/1.5` 和左右 `implicitSpringDamper`。用户随后确认两指同步张合、销轴运动中保持闭合；**无工件基础闭环运动已通过观察验证**，但该版本停止后抖动幅度、工件接触、夹持、抬升和重复循环尚未验收。

回退命令：

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py ag165_soft_mimic_test:=false
```

该模式保持 multiplier=1、右关节限位 `[-0.05, 0.93]`，恢复阻尼 0.20、显式 PD `2.5/0.35/2.5`，且不加入额外的隐式阻尼。插件在没有 `<offset>` 时继续采用启动采样偏置。两种模式都须查看 `/cr7as_ag165/mimic_status`，`followers=0/1` 时先修关节解析而非继续调符号。
## 2026-10-13 双侧接触传感器（旁路诊断）

`urdf/ag165_macro.xacro` 在 `ag165_link2_left/right` 的现有 STL collision 上新增两个 Gazebo Classic contact sensor，分别发布 `/ag165/contact_left_raw` 和 `/ag165/contact_right_raw`。闭环关节、PID、摩擦、CFM/ERP 未改。传感器引用的 `ag165_link2_left_collision` / `ag165_link2_right_collision` 是待 Ubuntu 展开验证的名称：用 `gz sdf -p` 查看实际生成的 SDF collision 和 sensor；若 Gazebo 没有话题或总是空状态，先核对 `<contact><collision>`，不能据此认定工件没碰到。完整状态过滤由 `cr7as_vision_motion` 的只读诊断节点完成。整块 STL 接触不等于独立指腹接触。


## 2026-10-16 Demo 附着物理切换插件

本包新增 `libdemo_attachment_physics_plugin.so`。它由红色工件的 `cr7as_vision_grasp/models/red_test_box/model.sdf` 加载，而库文件由本包安装到 `install/cr7as_grasp/lib/`。模型插件提供：

```text
/gazebo_grasp/set_demo_attachment_physics   std_srvs/srv/SetBool
```

`true`：在 Gazebo world-update 线程关闭红色工件 `red_box_link` 的 gravity 和 collision response，交由 Demo attachment follower 运动学跟随；`false`：恢复默认 collision mask 与 gravity。它不修改 AG165 SDF pin、mimic、PID、摩擦或 CFM/ERP。

每次更新后必须重新编译 `cr7as_grasp`，并彻底重启 Gazebo，不能让 gzserver 继续加载旧 `.so`：

```bash
colcon build --symlink-install --packages-select cr7as_grasp
ls ~/Desktop/dev_ws_sdf_gripper/install/cr7as_grasp/lib/libdemo_attachment_physics_plugin.so
```
