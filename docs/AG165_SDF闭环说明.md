# AG165-90 SDF 闭环说明

> **2026-10-12 结构真值更新**：本工作区已按 `H:\Desktop\AG165-90_4\ros2_humble\sdf\ag165_90_4_full_gripper.sdf` 重建 AG165 六个树关节，并添加同源的右主动关节 PD mimic Gazebo 插件。不要再使用“全部从动关节等于左主动关节”的 URDF mimic 简化模型；以根目录 README 和 `docs/Ubuntu迁移与验证.md` 的验证命令为准。

## 1. 目的

本工作区把大寰 AG165-90 夹爪安装到 CR7AS 末端，并在 Gazebo Classic 物理层保留四连杆闭环。ROS 2 / MoveIt 仍使用树状 URDF，Gazebo 侧再增加 CAD 销轴对应的 SDF revolute joint。

参考模型来源：

~~~text
H:\Desktop\AG165-90_4\ros2_humble\sdf\ag165_90_4_full_gripper.sdf
H:\Desktop\AG165-90_4\ros2_humble\meshes
~~~

交付后对应文件：

~~~text
src/cr7as_grasp/models/ag165_90_4/model.sdf
src/cr7as_grasp/models/ag165_90_4/meshes/
src/cr7as_grasp/urdf/ag165_macro.xacro
src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro
~~~

## 2. 机构拓扑

每侧由三个连杆和三个主要转动副组成，并由一个 Gazebo-only 销轴闭合：

~~~text
ag165_base_link
  ├─ ag165_joint1_left（主动）→ ag165_link1_left
  │    └─ ag165_joint2_left（被动）→ ag165_link2_left
  └─ ag165_joint3_left（被动）→ ag165_link3_left
       ↑ Gazebo 专用 ag165_left_link2_to_link3_sdf 销轴闭合 link2 与 link3
~~~

右侧对应 ag165_right_link2_to_link3_sdf。

### Pin 参数

- 相对父 link 的默认位置：0.022 0 0 m；
- `<pose relative_to="ag165_link2_left/right">` 必须明确写出；SDF 未指定时关节位姿默认相对子 link3，`0.022 0 0` 会落在错误坐标系。
- 旋转轴：0 0 1；
- 两个闭环关节默认启用；
- 左侧闭环 ODE 参数：cfm=1e-4、erp=0.2；右侧：cfm=1e-3、erp=0.2；
- 被动关节限位放宽到 [-3.14, 3.14]，避免限位墙和闭环约束互相抢占自由度。

## 3. 控制契约

只有 ag165_joint1_left 进入 ROS 2 夹爪控制器。以下关节不应加入 gripper_controller：

~~~text
ag165_joint1_right
ag165_joint2_left
ag165_joint2_right
ag165_joint3_left
ag165_joint3_right
~~~

右侧主动关节由 `libag165_gripper_mimic_plugin.so` 的 PD 力矩跟随左侧；四个被动关节由 Gazebo SDF 闭环求解，没有 URDF mimic。左侧主动关节由 `gazebo_ros2_control` 的位置 PID 以力矩驱动；保持现有 `FollowJointTrajectory` action。控制器的 `goal` 误差约束会防止关节不动却返回成功。首次 PID 参数为 `pos_kp=2.5`、`pos_ki=0`、`pos_kd=0.35`，须在 Ubuntu 实测后调整。

Humble 版 GazeboSystem 在同一个硬件组件内共用位置控制模式，因此 j1～j6 和 AG165 不应放入同一个 `<ros2_control>` 块。本工作区将机械臂放在 `GazeboArmSystem`，夹爪放在 `GazeboGripperSystem`；只有夹爪组件有位置 PID。`hold_joints=true` 的启动保持仅作用于有命令接口的关节，五个被动 AG165 关节没有命令接口，不会被这个选项锁住。机械臂 j1～j6 的 URDF 已有 1.0 的阻尼；启动塌臂不能归因于没有写阻尼。

0.20 rad 测试发现视觉交叠后，对照成功的独立 SDF 确认整机宏的 pin pose 曾缺少 `relative_to`。两侧 pin 坐标原本都定义在 link2 局部坐标系，已补齐该属性。原始 SDF 与本整机模型都关闭内部自碰撞，这有助于避免相邻铰链网格碰撞求解发散，也意味着 Gazebo 不会用自碰撞阻止可视网格交叠。修复 pin 坐标后如仍有问题，先比较零位和 0.20 rad 的关节反馈、pin 误差与 collision 显示，不要直接设 `self_collide=true`。

## 4. 独立 SDF 与完整机器人边界

models/ag165_90_4/model.sdf 是独立参考/检查模型；完整 CR7AS 运行入口使用 urdf/cr7as_ag165_gazebo.urdf.xacro。不要在完整场景中同时 spawn 独立 AG165 SDF 和 Xacro 集成的 AG165，否则会有两套夹爪和重复碰撞。

独立 SDF 的右侧 mimic 插件已作为源码编译到本工作区。源工程的 `ag165_hold_plugin` 只保持关节初始位置，不接收目标位置，也没有在完整 SDF 中使用；当前集成不加载它。

## 5. 调参原则

先确认闭环不炸模、不抖动，再确认主动关节反馈，之后小步调整 link2_left/right 的摩擦，最后调整工件摩擦、质量和 kp/kd。不要一开始使用极大摩擦系数，以免 ODE 求解器不稳定。

2026-10-12 左侧动、右侧不动并持续抖动时，不能直接采用“左 pin 阻尼 0.005、CFM 5e-4、ERP 0.3、左主动阻尼 0.15”的整套建议。原参考 SDF 的左主动阻尼是 0.08，左 pin 是 CFM 1e-4 / ERP 0.2，右 pin 是 CFM 1e-3 / ERP 0.2；`0.022 0 0` 本身不证明有 0.5 mm 装配误差。先定位右侧跟随插件是否加载、右关节名称是否解析、右侧是否收到力矩以及闭环是否几何死锁。

插件现提供 `/cr7as_ag165/mimic_status` (`std_srvs/srv/Trigger`)：重新编译并重启 Gazebo 后可读 `active found`、`followers`、`enabled`、左右主动关节位置、右侧目标和最后一次施加力。`/cr7as_ag165/mimic_enable` 存在只表示插件 ROS 服务创建成功；新版在关键关节解析失败时，对启用请求返回 `success=false`。详细 Ubuntu 命令与判读见工作区根 `README.md`。

