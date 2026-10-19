# 文档导航

本目录只保留与 AG165-90 建模、闭环仿真和迁移直接相关的说明。推荐阅读顺序：

1. [官方上游参考](官方上游参考.md)：分清官方大寰仓库、第三方 ROS 2 移植和本项目的边界。
2. [AG165 SDF 闭环说明](AG165_SDF闭环说明.md)：了解当前的机构、pin 坐标和控制器契约。
3. [架构、坐标与故障复盘学习指南](AG165_SDF闭环_架构坐标与故障复盘学习指南.md)：学习 URDF/SDF 分层、排障与真机迁移边界。
4. [Ubuntu 迁移与验证](Ubuntu迁移与验证.md)：在 ROS 2 Humble / Gazebo Classic 环境中构建、启动和验收。
5. [交付清单与当前生效入口](交付清单与当前生效入口.md)：识别当前运行入口与历史资料。

`历史备份文件说明.md` 用于避免把旧模型或旧配置误当作当前生效版本。

> **2026-10-19 当前优先 Demo：** 抓取附着后，垂直朝下到 TCP `[0.10, 0.48, 0.36] m`，在框口上方开爪并解除附着，观察红块落框后撤离；不执行原深下降。
> 新开关 `demo_drop_release_enabled: true` 已写入 YAML。历史证据仅为空爪终点有效，带件搬运和开爪仍由运行时重新检查。
> 同步文件、编译、完整服务和日志命令见 [Demo 框上释放同步验证](docs/Demo_垂直朝下框上释放_同步验证.md)。30 mm 微抬升服务仍只验证微抬升。

> **当前生效的场景和深框几何规范：**
> [`docs/CR7AS_AG165_四层统一标准_v1.2_生效基线.md`](docs/CR7AS_AG165_四层统一标准_v1.2_生效基线.md)。
> `v1.1` 仅保留调试/试验历史，不能作为运行参数来源。
> 已执行深框实验请追加到：[`docs/深框工艺验证与迭代日志.md`](docs/深框工艺验证与迭代日志.md)。
> 梅卡曼德公开方法的无序深框抓取映射见：[`docs/梅卡曼德参考_无序深框抓取SOP.md`](docs/梅卡曼德参考_无序深框抓取SOP.md)。

## 2026-10-13 AG165 双侧接触旁路诊断（Ubuntu 待验证）

在 `ag165_macro.xacro` 的左右 `ag165_link2` **现有整块 STL collision** 上增加 Gazebo Classic contact sensor；`demo_pick_place.launch.py` 同时启动只读 `/ag165_contact_diagnostics`。原始话题 `/ag165/contact_left_raw`、`/ag165/contact_right_raw` 的类型为 `gazebo_msgs/msg/ContactsState`，过滤和时间窗口结果在 `/vision_guided_motion/contact_diagnostics`（JSON 字符串）。诊断节点记录首见的完整碰撞体名称、工件白名单接触、接触点 world/TCP 坐标、左右关节角、位置差分速度、原始 velocity 和控制器误差。**不改控制器、PID、摩擦、CFM/ERP，不向执行器状态机反馈，更不会调用 attach/lift。** 当前 `link2` 不是独立指腹 collision，匹配工件也不证明指腹贴合。`<collision>` 的 `_collision` 名称必须在 Ubuntu 用展开 SDF 和原始 contact 话题验证；若不匹配会没有数据，不能据此判定“无接触”。

迁移后构建 `cr7as_grasp cr7as_vision_motion`，完全重启 `ros2 launch cr7as_vision_motion demo_pick_place.launch.py`；先查看 `ros2 topic list | grep ag165/contact`、`ros2 topic echo /ag165/contact_left_raw --once` 和右侧，再看诊断话题。传感器可能只在有碰撞时发送有效状态；`--once` 可能一直等待，另开终端观察并用 Ctrl+C 停止。**不要在此阶段调用完整抓取或抬升**；只运行已验证的 `/vision_guided_motion/execute_contact_test` 收集双指接触数据。详情见 `src/cr7as_vision_motion/README.md`。

## 2026-10-13 阶段 1：同位姿空载夹爪基准（待 Ubuntu 验证）

执行器新增 `contact_test_stop_after_approach`（默认 `false`）。只对 `/vision_guided_motion/execute_contact_test` 生效：正常执行预抓取、clearance、最终 approach，并确认 Gazebo 工件未明显横移，然后停在最终 TCP 位姿，**不闭爪、不 attach、不抬升、不退回**。目的是在相同机械臂姿态下移走仿真方块，单独命令夹爪从 0 到已验证的 `0.62 rad`，观察是否仍在约 `0.18/0.42 rad` 出现速度骤降。终端录像应同时包含高频 `/joint_states` 与 `/gripper_controller/controller_state` 的目标/反馈，以分析位置、速度和跟踪误差；前次 10 Hz 左右的 controller_state 墙钟采样不能单独判定 0.25 s 停滞。保持 AG165 PID、SDF 闭环 CFM/ERP、摩擦等物理参数不变。Gate 后对象接触许可暂留启用且机器人可能邻近工件，**不可继续启动 MoveIt 运动或第二次抓取**；仅在 Gazebo 仿真中删除工件做直接夹爪 Action，实验后完整重启仿真。具体步骤见 `src/cr7as_vision_motion/README.md`。

这是交付给 Ubuntu / ROS 2 Humble 的 **CR7AS 六轴机械臂 + 大寰 AG165-90 夹爪** Gazebo Classic 仿真工作区。目标是：复制整个目录到 Ubuntu 后，按文档完成编译、启动、夹爪闭环验证、视觉定位、MoveIt 轨迹规划、碰撞检测、抓取附着、运输和放置。

> 原始 `H:\Desktop\rosstudy\dev_ws` 未修改。本目录是独立交付副本；Windows 端只做源码/XML/路径检查，最终的 `colcon build`、Xacro、Gazebo 和 MoveIt 验证必须在 Ubuntu ROS 2 Humble 执行。

> **当前默认配置是正向跟随的低增益模式；用户已确认基础闭环运动通过，但完整抓取尚未验证。** 反向系数 `-1` 曾导致一侧张开、一侧闭合，现已改回 `+1`。参考 SDF 参数可用 `ag165_soft_mimic_test:=false` 回退。

> **下一阶段先运行接触测试，不直接运行完整搬运。** 新服务 `/vision_guided_motion/execute_contact_test` 只执行视觉定位、预抓取、接近、闭爪、检查工件位移，然后打开夹爪并退回；不附着、不抬升、不去料箱。Ubuntu 端尚未对该新服务完成编译和实测。

> **2026-10-13 相机视场调整（待 Ubuntu 验证）：** 完整 AG165 Gazebo Xacro 的虚拟相机由 `base_link` 中 `(0, 0.50, 1.20)` 改为 `(0, 0.80, 1.60)` m，水平视场由约 60° 改为 80° (`horizontal_fov=1.3963` rad)。桌面顶面 `z=0.62` m 处的理论覆盖范围约为 `1.64 × 0.93` m，可包含 `1.20 × 0.80` m 桌面；这只是无遮挡、相机正对桌面的几何估算，须检查 Gazebo 实际画面和机械臂遮挡。`plane_z_m` 保持 `0.62` m，图像仍为 `1280×720`。

> **2026-10-13 P2 崩溃修复（待 Ubuntu 验证）：** `pixel_to_table_pose.py` 曾把 `PoseStamped` 传给 Humble 的 `tf2_geometry_msgs.do_transform_pose()`，在收到 `/vision/detected_pose_2d` 后报 `AttributeError: 'PoseStamped' object has no attribute 'position'` 并退出。现改为传入 `PoseStamped.pose`，读取返回的 `Pose.orientation`。同步后需重新编译 `cr7as_vision_grasp`、source 并重启完整场景；此修复不修改相机参数或 `plane_z_m`。

## 当前生效链路

2026-10-13 正式抓取规划的 ACM 修复（**待 Ubuntu 重编译验证**）：Ubuntu 在 `grasp_clearance_m=0.05` 时已实际到达 TCP z≈0.70 m，Gazebo 工件 3D 位移为 0；关闭 clearance 门控后，日志在设置 `red_test_box`/夹爪接触许可的紧接下一步报告 `l1`–`l2` 自碰撞、`Start state appears to be in collision`，而非证实最终 z=0.65 不可达。根因是执行器原先把仅含工件/夹爪的**稀疏 AllowedCollisionMatrix**作为 PlanningScene diff 提交，覆盖了 MoveIt 从 SRDF 载入的 `l1`–`l2` 相邻关节豁免。现改为先从 `/get_planning_scene` 读取**完整实时 ACM**，核验 `l1`–`l2` 豁免仍在，仅改 `red_test_box` 与配置的夹爪 link 对，再提交完整 ACM；恢复接触限制时也同样保留全矩阵。读取或应用失败则不继续下降。工件、桌面、料箱及其余机械臂碰撞仍启用。只同步 `src/cr7as_vision_motion/src/pick_place_executor.cpp`、该包 `CMakeLists.txt` 和 `package.xml`（以及此 README），重编译本包并**完全重启含 move_group 的完整 launch**，再运行无附着 `execute_contact_test`。若出现新的碰撞对，应按新日志处理，不直接进入完整抓放。

2026-10-13 接触测试通过后的微抬升门控（**待 Ubuntu 验证**）：实测 `/vision_guided_motion/execute_contact_test` 返回 `Contact test complete: stall inferred, object stayed stable; gripper opened and robot retreated.`。这只证明停滞推断与位移阈值通过，不证明双指接触或实体夹持。新增 `/vision_guided_motion/execute_pick_lift_test`：沿用预抓取、clearance、正式接近、闭爪停滞、Gazebo attach，随后只上抬 `pick_lift_test_distance_m=0.03` m，核对 Gazebo 工件世界位移与名义抬升相差不超过 `pick_lift_test_follow_tolerance_m=0.005` m，**到此停止，不去料箱、不 detach、不张爪**。对象会保持附着，检查结束后应完整重启仿真以复位，不能直接重复调用。由于当前 attachment follower 是运动学跟随，这只是 follower/动作链验证，**不是实际摩擦抓牢或 TCP 相对滑移的物理证明**。运行命令和限制见 `src/cr7as_vision_motion/README.md`。

2026-10-13 AG165 开合方向实测纠偏（待本次 Windows 修改在 Ubuntu 编译验证）：Ubuntu `/gripper_controller/controller_state` 显示启动左主动关节约 `-0.0007 rad`，手动命令 `0.62 rad` 后实际约 `0.6198 rad`，用户目视确认这是**闭合**而非张开。此前执行器/YAML/SRDF 将 `0.62` 错作 open、`0.05` 错作 close，会在预抓取前提前闭爪，属于抓取流程安全错误。现统一改为 open=`0.0 rad`（与启动开姿态一致）、close=`0.62 rad`（由接触停滞监测提前取消，不保证目标一定可达），仅在有新鲜且基本静止的反馈表明已经张开时跳过重复开爪 Action。**未**把 `0.93 rad` 的 URDF 极限当作安全闭爪目标，也未单独修改 SDF 闭环的初始连杆位姿。下一步先用已有的 `contact_test_stop_after_clearance=true` 与 `grasp_clearance_m=0.05` 仅验证第一段实际下降和工件 3D 位移；通过后再做不附着的接近/闭爪接触测试，最后才做附着、抬升、深框放置。当前料箱底面 z=0.66、墙顶 z=0.80、放置 TCP z=0.692；按 MoveIt 附着方块中心假设，方块底面 z≈0.662，仅离底面 2 mm，需结合真实 Gazebo 工件相对 TCP 位姿核对，不能直接声称深框放置安全。

2026-10-13 clearance probe 实测与下一门控：Ubuntu 在预抓取状态、yaw=90° 下得到 offset 0.03/TCP z=0.68 规划失败，而 0.05/0.08/0.10（TCP z=0.70/0.73/0.75）均规划成功；这只说明从该关节状态到这些目标可规划，不等于执行后不会碰动工件，也不证明最终抓取位 z=0.65 安全。当前 Windows 执行器源码另有 `contact_test_stop_after_clearance`（默认 false）和 `max_clearance_object_shift_m=0.005`：当仅接触测试模式打开该门控时，程序执行到 clearance 后读取 Gazebo 工件位姿，3D 位移超过 5 mm 即失败，否则成功停止，**不进入最终 approach、闭爪或 attach**。Ubuntu 是否已包含这段代码，应以同步文件、重编译并重启后 `ros2 param get /pick_place_executor contact_test_stop_after_clearance` 可查询为准。下一轮只将 `grasp_clearance_m` 临时设为 0.05、预抓取门控设 false、clearance 门控设 true，运行 `/vision_guided_motion/execute_contact_test`；观察 Gazebo 实体与状态日志，别直接调用完整抓放。若实体位移超过阈值或发现碰撞，停止并校准碰撞几何/TCP。见执行器包 README。

2026-10-13 AG165 clearance 规划诊断（待 Ubuntu 验证）：在预抓取成功后，日志返回 `MoveIt plan result code: 99999` 和 `Unable to sample any valid states for goal tree`，因此是 `Grasp clearance` **规划**失败，不是 Gazebo 工件状态服务或轨迹执行失败。当前抓取 TCP z=0.65 m、clearance TCP z=0.68 m；后者约与工件顶面齐平，但尚无碰撞对证据，不能把 OMPL 错误单独解释成指尖碰撞。执行器现在把 clearance 规划/执行错误分开，并新增 `/vision_guided_motion/probe_grasp_clearance`：只从机器人当前关节状态规划配置的几个 clearance 候选，保留工件在 MoveIt 场景中，**绝不执行机械臂、夹爪或 attach**。另可将 `contact_test_stop_after_pregrasp=true`，使接触测试只移动到预抓取位便返回，不再执行下降或闭爪；默认 false 保持原接触测试语义。参数在 `src/cr7as_vision_motion/config/pick_place.yaml`，默认 probe yaw=90°、offset=0.03/0.05/0.08/0.10 m；只有确认预抓取所选 yaw 与当前关节状态后才解读结果。诊断成功只说明路径可规划，不证明真实几何接触或后续抓取安全；不要据此直接触发完整抓放。详见执行器包 README。

2026-10-13 Gazebo 工件状态服务修复（待 Ubuntu 验证）：预抓取已成功规划和执行，但接触测试在最终下探前报告 `Gazebo object-state service is not available`，并返回 `Could not capture Gazebo object position before final approach.`。根因是 AG165 启动文件使用通用 `gazebo.launch.py` 的默认 world，未加载 `libgazebo_ros_state.so`；旧 CR7AS 启动文件使用 `worlds/empty_with_state.world`。现 AG165 启动文件也通过 `gzserver.launch.py` 加载该 world，并通过 `gzclient.launch.py` 启动界面。此修复不更改夹爪 SDF、机械臂控制器或视觉位姿。同步 `src/cr7as_grasp/launch/gazebo_ag165_control.launch.py` 后完全退出旧 Gazebo、重编译 `cr7as_grasp`、重新启动完整场景。触发测试前先确认 `/get_entity_state` 与 `/set_entity_state` 均存在，并实际调用前者查询 `red_test_box` 得到 `success=True`；仅有 `/spawn_entity` 或能看见红色工件不足以说明状态插件已加载。若服务仍缺失，检查 gzserver 启动日志中的 world 路径和 `gazebo_ros_state` 插件加载错误，不要绕过安全读取。

2026-10-13 预抓取规划对照修复（待 Ubuntu 验证）：新日志的四个 yaw 候选都先报 `l6` 与 `ag165_base_link` 自碰撞，随后 `Start state appears to be in collision` / `Skipping invalid start state`，因此当前失败不是目标位姿超出工作空间的证据。两者经 `l6 -> gripper_base -> ag165_base_link` 固定安装；新 SRDF 只排除了相邻两对，却漏了安装处实际相交的 `l6-ag165_base_link`。现仅加入这一个固定安装碰撞豁免，不恢复旧 AG95 SRDF 中大批 `Never` 条目，不关闭工件/桌面/其他机械臂碰撞检查。另对照发现 AG165 整机 Xacro 把原 CR7AS 的 j1/j2/j4/j5/j6 `[-3.14,3.14]`、j3 `[-2.79,2.79]` 均误收窄为 `[-1.57,1.57]`；已恢复旧工作区六轴原限位。机械臂的 b0～l6 网格、几何和 `grasp_tcp` 固定安装位置未改。同步本次 Xacro 与 SRDF 后在 Ubuntu 重编译 `cr7as_grasp cr7as_grasp_moveit_config`、完全重启 `demo_pick_place.launch.py`，再测 `/vision_guided_motion/execute_contact_test`。若出现新的碰撞对或 IK 失败，以新日志继续定位，不能据此宣称预抓取已经通过。

2026-10-13 起始自碰撞修复：接触测试四个预抓取候选均报告 `l1` 与 `l2` 碰撞，OMPL 无法初始化起始树。AG165 版 SRDF 漏了原 CR7AS 的六组相邻连杆排除；现恢复 `b0-l1`、`l1-l2`、`l2-l3`、`l3-l4`、`l4-l5`、`l5-l6`，已逐一核对 URDF 父子关系及原工作区 SRDF。未恢复旧文件中批量 `Never` 排除，也未关闭环境碰撞检测。同步 `src/cr7as_grasp_moveit_config/config/cr7as_grasp.srdf` 后重新构建 `cr7as_grasp_moveit_config` 并完全重启完整 launch，使 MoveIt 重新加载 SRDF。Windows XML/拓扑核对通过，Ubuntu 规划仍待验证；若仍失败，应按新日志中的实际碰撞对继续排查，不能据此保证所有候选可达。

2026-10-13 执行器同步修复：Ubuntu 启动日志仍打印旧版 `Pick/place executor created. Call ...execute_pick_place`，与当前含接触测试服务的源码不一致。最新执行器打印 `Executor build contact-test-20261013`，并在 MoveIt 初始化后打印 `starting executor`。完整 launch 移除了执行器的冗余全局节点名重映射，节点本身仍叫 `/pick_place_executor`。规划桌面中心已从错误的 z=0.30 改为 z=0.60，与场景中厚度 0.04、顶面 z=0.62 对齐。此修正可能使以前错误通过的轨迹被碰撞检查拒绝，不应关闭碰撞检查来通过测试。

同步本次 C++、launch、CMakeLists.txt 和 package.xml 后，停止旧完整 launch，运行：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr7as_vision_motion --cmake-clean-cache
source install/setup.bash
strings install/cr7as_vision_motion/lib/cr7as_vision_motion/pick_place_executor | grep -F 'contact-test-20261013'
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
```

只有构建成功且二进制包含版本标记才启动。另一个 source 后的终端检查 `ros2 node info /pick_place_executor` 的 Service Servers 和 `ros2 service list | grep execute_contact_test`。先确认 `/get_entity_state` 可查询工件，再触发接触测试。Windows 未执行 ROS 2 C++ 编译或 Gazebo 运行；Ubuntu 验证仍待完成。新补丁不含 YAML，不覆盖 Ubuntu 上的抓取高度校准。

```text
cr7as_ag165_gazebo.urdf.xacro
  ├─ CR7AS 六轴机械臂
  ├─ ag165_macro.xacro
  │    ├─ AG165 网格、惯量、碰撞和摩擦
  │    ├─ URDF 运动学树
  │    └─ Gazebo-only 两个 SDF revolute pin 闭环
  ├─ GazeboArmSystem（j1～j6，直接位置控制）
  ├─ GazeboGripperSystem（左主动关节位置 PID，其他关节状态反馈）
  ├─ gazebo_ros2_control
  └─ grasp_tcp / virtual_camera

spawn_grasp_scene.launch.py
  ├─ Gazebo + CR7AS + AG165
  ├─ table / place_bin / red_test_box
  └─ attachment_follower

 demo_pick_place.launch.py
  ├─ MoveIt move_group
  ├─ RGB detector + 二维平面位姿
  └─ pick_place_executor
       detect -> pre-grasp -> approach -> close -> contact check
       -> Gazebo attach confirmation -> lift -> transport -> place -> detach
```

## 重要路径

- `src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro`：完整机器人 Gazebo robot description。
- `src/cr7as_grasp/urdf/ag165_macro.xacro`：AG165 的 link、mesh、碰撞、摩擦、右侧力控跟随和 SDF 闭环。
- `src/cr7as_grasp/models/ag165_90_4/model.sdf`：独立 AG165 SDF 参考模型；**不与整机 Xacro 同时 spawn**。
- `src/cr7as_grasp/meshes/ag165/`：完整 AG165 STL 网格。
- `src/cr7as_grasp/launch/gazebo_ag165_control.launch.py`：唯一推荐的 AG165 单机 Gazebo 启动入口。
- `src/cr7as_grasp/config/ag165_gazebo_controllers.yaml`：ros2_control 配置。
- `src/cr7as_grasp_moveit_config/config/`：MoveIt robot description、SRDF 和控制器映射。
- `src/cr7as_vision_grasp/launch/spawn_grasp_scene.launch.py`：完整场景入口。
- `src/cr7as_gazebo_grasp/cr7as_gazebo_grasp/attachment_follower.py`：抓取确认后按 TCP 相对位姿跟随工件的 Gazebo MVP 节点。
- `src/cr7as_vision_motion/src/pick_place_executor.cpp`：视觉抓取、夹爪 action、规划场景和 attachment 状态机。

## 虚拟相机视场与 Ubuntu 验证

当前相机参数位于 `src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro` 的 `base_to_virtual_camera_joint` 和 `virtual_camera_sensor`。提高图像分辨率不会改变视场；本次只更改相机位置及 `<horizontal_fov>`，不更改光学坐标系、机器人 TCP、工件/桌面高度或视觉投影平面。

把上述 Xacro 同步到 Ubuntu，关闭旧 Gazebo 后运行：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr7as_grasp
source install/setup.bash
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
```

另开终端运行 `ros2 run rqt_image_view rqt_image_view`，先选 `/virtual_camera/virtual_camera/image_raw` 核对桌面是否完整入镜，再选 `/vision/debug_image` 核对红色工件仍能被检测。视场扩大后目标像素面积会变小，若检测失效先测 `/vision/detection_valid` 和轮廓面积，再决定是否调整检测阈值，**不要**凭视觉窗口缩放判断相机视场。确认 `/virtual_camera/virtual_camera/camera_info` 持续发布，P2 才能使用更新后的相机内参；`plane_z_m=0.62` 不应因相机升高而改动。旧工作区 `H:\Desktop\rosstudy\dev_ws` 未修改。

P2 修复同步到 Ubuntu 后，执行 `colcon build --symlink-install --packages-select cr7as_vision_grasp`，重新 `source install/setup.bash` 并重启完整场景。验证 `ros2 node list | grep pixel_to_table_pose` 存在、`ros2 topic echo /vision/object_pose_base --once` 有输出，再运行接触测试；节点不再崩溃只证明视觉投影链恢复，**不等于抓取成功**。

## AG165 闭环与控制器契约

AG165 的 ROS/MoveIt 模型保持树结构；Gazebo 物理层补充两条 pin 约束：

- `ag165_left_link2_to_link3_sdf`：`ag165_link2_left -> ag165_link3_left`。
- `ag165_right_link2_to_link3_sdf`：`ag165_link2_right -> ag165_link3_right`。
- 默认 pin 坐标：`0.022 0 0` m；默认轴：`0 0 1`。
- 左右 pin 的 `<pose>` 必须分别带 `relative_to="ag165_link2_left"` 和 `relative_to="ag165_link2_right"`。数值是各自 link2 的局部坐标；漏写时 SDF 默认按子连杆 link3 解释，闭环会错位。
- 默认 ODE 参数：左 pin `cfm=1e-4`，右 pin `cfm=1e-3`；两侧 `erp=0.2`。
- 只有 `ag165_joint1_left` 进入 `gripper_controller`；`gazebo_ros2_control` 对该关节使用位置 PID，将轨迹目标转成物理力矩。
- `gazebo_ros2_control` 的 `hold_joints=true` 在控制器启动前保持有命令接口的机械臂关节；五个只有状态接口的 AG165 从动关节仍由物理闭环求解。
- `ag165_joint1_right`、`ag165_joint2_left/right`、`ag165_joint3_left/right` 不发送 ROS 2 trajectory goal，由 mimic + Gazebo SDF 闭环处理。

因此当前**不新增第二个 AG165 controller**。SDF 闭环是物理约束，不是另一个 ROS 2 controller。不要把全部六个夹爪关节塞进同一个 trajectory controller，否则容易和闭环过约束。

2026-10-12 运行反馈：`ag165_joint1_left/position [claimed]`、`/cr7as_ag165/mimic_enable` 均存在，但 action 反馈中 `desired≈0.20 rad`、`actual≈-0.007 rad`，说明原位置接口未带动闭环。现为左主动关节设置 `pos_kp=2.5`、`pos_kd=0.35`（积分为零），使 GazeboSystem 使用力矩 PID；夹爪控制器增加轨迹和终点误差约束。参数是保守起点，必须在 Ubuntu 观察实际运动、抖动和温和接触后调节。`Goal successfully reached` 只有与真实 `/joint_states` 一致时才算通过。

同日启动塌臂复盘：Humble 的 GazeboSystem 在单个硬件组件中共用 `position_control_method_`。上一版把六轴和带 PID 的夹爪放在同一组件，导致六轴在接管后也进入 PID 模式，却没有各自的增益；同时 `hold_joints=false`，控制器启动前也不保持六轴。已拆成 `GazeboArmSystem` 和 `GazeboGripperSystem` 两个组件，恢复 `hold_joints=true`，并将整机 spawn 高度从 0.10 m 恢复为原机械臂的 0.0 m。机械臂 URDF 的 j1～j6 均已有 `damping=1.0`，从动夹爪关节只有 state interface；日志中的 `Command:` 标题不代表存在命令接口。

随后 0.20 rad 单机测试出现夹爪连杆视觉交叠。检查发现整机 Xacro 的 Gazebo 闭环 pin `<pose>` 漏了参考 SDF 中的 `relative_to="link2_*"`；已按完整机器人前缀补齐。参考 SDF 本身也使用 `self_collide=false`，所以截图中的相交不能单凭画面判定为碰撞体穿透。先验证最终转换的 SDF 中 pin 位置，再看 action 的 actual 位置、左右从动关节状态和 Gazebo 中的 collision 可视化；不要直接开启全部自碰撞。

### 左右不同步、持续抽搐的诊断（2026-10-12）

`0.022 0 0` 是 pin 在各自 link2 中的坐标，不等于已经测得 0.5 mm 装配误差。若左侧到达 0.20 rad、右侧不动且持续抽搐，**先查右侧 PD 跟随插件是否找到右关节并实际施力**，不要同时把左 CFM、ERP、主动关节阻尼改成未经验证的数值。当前宏保持参考 SDF：左主动关节 damping=0.08，左 pin cfm=1e-4 / erp=0.2，右 pin cfm=1e-3 / erp=0.2。

本版插件新增状态服务；在 Ubuntu 重新编译 `cr7as_grasp`、重新 source 并**完全重启 Gazebo**后运行：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /cr7as_ag165/mimic_status std_srvs/srv/Trigger '{}'
ros2 service call /cr7as_ag165/mimic_enable std_srvs/srv/SetBool '{data: true}'
```

`mimic_status` 会报告 `active found`、`followers=1/1`、`enabled`、左右主动关节位置、右关节目标和实际施加的力。仅看到 `/cr7as_ag165/mimic_enable` 服务**不能证明**右关节已解析成功。请分别在静止、执行 0.20 rad 期间、执行后读取状态，并对照 `/joint_states` 的 `ag165_joint1_left/right`：

- `found=false` 或 `followers=0/1`：优先检查 Gazebo 实际加载的关节名称及插件加载日志；单独调阻尼无效。
- `followers=1/1` 但 `enabled=false`：调用 `mimic_enable`；新版本找不到关节时启用请求会返回失败。
- `target` 变化但右侧 `position` 不变、`force` 长期接近当前试验上限 1.5：优先排查右侧销轴坐标、轴线、转换后的 SDF、物理死锁和电机力矩，不要把“右侧不动”归咎于左侧阻尼。
- 左右关节位置都跟随但视觉上仍脱节：查看 Gazebo 实际 link/collision 位姿和 pin 误差；JTC 的 SUCCEEDED 只验证左主动关节。

若右侧驱动与 pin 坐标均已确认正确而仍有小幅振荡，再以**一次只改一个参数**的方式小步试验阻尼或 CFM/ERP；记录静止和动作后的左右关节位置/速度与 pin 间隙，不能以肉眼“没抖”代替闭环精度检查。增大 CFM 会柔化约束，也可能让 pin 间隙增大；ERP 提高不保证更稳定。

参考 SDF 的右侧 `multiplier=1.0`、右主动关节限位 `[-0.05, 0.93]`。Ubuntu 对照结果：`multiplier=-1` 后启动抽搐消失，但 0.20 rad 时两指呈相反开合；这是反号配置不适合当前装配方向的直接证据。抖动消失可能来自同时改变的低增益、阻尼、隐式阻尼或 offset，**尚不能单独归因**。

随后用户确认：改回 `multiplier=+1` 后，两指同步张合，运动时两侧销轴保持闭合。**因此无工件的基础 SDF 闭环运动测试通过**。这项确认是用户的 Gazebo 观察反馈；尚未取得该版本停止后抖动幅度、定量 pin 间隙、重复循环、夹持工件或抬升后的数据，不能等同于完整视觉抓取验收。

### 正向跟随低增益试验（默认开启，可回退）

`gazebo_ag165_control.launch.py` 现使用 `ag165_soft_mimic_test` 参数，默认 `true`。试验模式保留上一轮可能抑制抖动的显式 offset=0、右主动关节阻尼 0.15、右侧 PD `kp=1.5/kd=0.3/effort=1.5` 和左右主动关节 `implicitSpringDamper`，但把右侧 multiplier **单独改回 +1**，右侧限位也恢复参考值 `[-0.05, 0.93]`。插件没有 `<offset>` 标签时仍保留原启动采样行为。**仅用于无工件的低速小角度验证；暂不执行完整抓取。**

`mimic_status` 同时报告实际加载的 `multiplier/offset/kp/kd/effort_limit`。应先确认看到 `+1/0/1.5/0.3/1.5`，再比较左右关节反馈；服务路径是 `/cr7as_ag165/mimic_status`，不是 `/<节点名>/mimic_status`。

把修改后的完整工作区同步到 Ubuntu 后，关闭旧 Gazebo，在工作区根目录编译本包并重新启动：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr7as_grasp
source install/setup.bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py
```

另一终端先查状态，再低速发 `0.05 rad / 5 s`，确认左右关节实际位置变化且 pin 不脱节后，才测试 `0.20 rad / 5 s`：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 service call /cr7as_ag165/mimic_status std_srvs/srv/Trigger '{}'
ros2 action send_goal --feedback /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [ag165_joint1_left], points: [{positions: [0.05], time_from_start: {sec: 5, nanosec: 0}}]}}"
```

动作**进行中**在另一个终端再次调用 `mimic_status`。若右侧仍不动或抖动加剧，不要继续 0.20 rad，保存状态输出并回退。

回退到参考 SDF 的正向跟随配置，不需要手改文件：

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py ag165_soft_mimic_test:=false
```

回退模式保持 multiplier=1、右关节限位 `[-0.05, 0.93]`，恢复启动采样 offset、阻尼 0.20、右侧 PD `2.5/0.35/2.5`，并关闭新增的隐式阻尼标签。请分别在 `true` 与 `false` 模式记录 `/cr7as_ag165/mimic_status` 静止及低速动作中的 `target/position/force`。若 `followers=0/1`，改系数无法驱动右关节，应先修复关节名称/插件加载问题。

## 2026-10-12：SDF 真值重建修正

`urdf/ag165_macro.xacro` 已不再使用旧 SW 导出链的错误关节坐标。它现在逐项采用：

```text
H:\Desktop\AG165-90_4\ros2_humble\sdf\ag165_90_4_full_gripper.sdf
```

中的左右 `joint1/joint2/joint3` 原点、轴、惯量与左右 pin 闭环参数。两种模式的右主动关节限位与跟随系数均恢复参考值；当前默认低增益模式只调整阻尼、PD、offset 和隐式阻尼，尚不能称为与参考 SDF 完全相同。原始 STL 仅作为几何资源，已经过 SHA256 校验，**不是模型散架的根因**。

同时新增 `libag165_gripper_mimic_plugin.so`：它按成功 SDF 的方式，以力控 PD 令右侧 `ag165_joint1_right` 跟随左侧主动关节；其余四个被动关节只暴露 Gazebo state interface，不进入 trajectory controller，也不再使用不真实的 `multiplier=1` URDF mimic。

每次 Ubuntu 编译后必须确认该插件存在：

```bash
ls ~/Desktop/dev_ws_sdf_gripper/install/cr7as_grasp/lib/libag165_gripper_mimic_plugin.so
```
## 下一阶段：工件接触测试与附着前位移门

`cr7as_vision_motion` 新增 Gazebo `GetEntityState` 依赖。正式执行和接触测试都会记录工件在**最终下降前、下降后、闭爪后**的世界坐标；若任一阶段的横向位移超过 `max_object_lateral_shift_m=0.015`，停止并尝试张爪/退回，**不调用 attach**。夹爪位置/速度反馈缺字段、超过 `max_gripper_feedback_age_sec=0.5` 秒或非有限数时不用于停滞推断；推断停滞后还必须确认夹爪 action 已取消并结束，不能在闭爪目标仍执行时继续附着。正式执行还要求闭爪时推断到关节停滞；单凭 action `SUCCEEDED` 或接近目标中心不能证明物理接触。当前停滞判据仅是仿真近似，不能代替双指接触/力反馈。

先将更新后的 `cr7as_vision_motion` 包同步到 Ubuntu，退出 conda base（如仍激活），在 ROS 2 Humble 环境编译并**完全重启完整场景**：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr7as_vision_motion
source install/setup.bash
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
```

另开终端观察状态，再调用接触测试：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 topic echo /vision_guided_motion/pick_place_status
# 另开终端：
ros2 service call /vision_guided_motion/execute_contact_test std_srvs/srv/Trigger '{}'
```

先确认 `/get_entity_state` 可用。记录 `Grasp approach object lateral shift`、`Contact stage: object lateral shift`、`gripper stall inferred` 及接触测试最终响应。正常接触测试不会调用 `/gazebo_grasp/attach`，也不会抬升或搬运；结束后自动张爪并规划退回预抓取位。**若退回失败，服务会报错，须停止并人工检查，不要重复触发。** 只有接近阶段未推走工件、闭爪阶段位移未超阈值且停滞反馈合理，才继续验证原 `/vision_guided_motion/execute_pick_place` 的 attach、抬升与放置。单次通过不等于重复可靠，也不证明接触来自双指而非机构卡滞。

关键参数位于 `src/cr7as_vision_motion/config/pick_place.yaml`：`gazebo_object_state_service`、`gazebo_object_model`、`gazebo_world_frame`、`max_object_lateral_shift_m`、`contact_test_hold_sec`、`require_gripper_stall_for_attachment`。0.015 m 只是当前演示的保护阈值，不是工件的真实工艺公差。

## Ubuntu 迁移和编译

把整个目录复制到 Ubuntu，例如 `~/Desktop/dev_ws_sdf_gripper`，然后：

```bash
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
```

推荐依赖：

```bash
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro \
  ros-humble-tf2-geometry-msgs
```

## 首次验证顺序

### 1. Xacro 和 URDF

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 run xacro xacro \
  ~/Desktop/dev_ws_sdf_gripper/src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro \
  use_sdf_closure:=true > /tmp/cr7as_ag165.urdf
check_urdf /tmp/cr7as_ag165.urdf
```

检查结果必须包含：`ag165_joint1_left`、`ag165_joint1_right`、`ag165_link2_left`、`ag165_link3_left`、`ag165_link2_right`、`ag165_link3_right`、`grasp_tcp`。

### 2. 只启动机器人和夹爪

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states
```

张开：

```bash
ros2 action send_goal \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [ag165_joint1_left], points: [{positions: [0.62], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

首次先将目标改为 `0.20`，`time_from_start` 改为 5 秒，用 `--feedback` 观察 desired/actual；确认运动后再试 `0.62`，闭合目标为 `0.05`。必须看到主动关节位置/速度变化、左右机构同步、无 NaN、无明显抖动或炸模。详细命令见 `docs/Ubuntu迁移与验证.md`。

### 3. 完整视觉抓取

```bash
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
```

检查：

```bash
ros2 node list | grep -E 'gazebo|move_group|pick_place|attachment|rgb|pixel'
ros2 service list | grep -E 'gazebo_grasp|attach|detach|get_entity_state|set_entity_state'
ros2 topic echo /vision_guided_motion/pick_place_status
ros2 topic echo /gazebo_grasp/attachment_status
```

触发：

```bash
ros2 service call /vision_guided_motion/execute_pick_place std_srvs/srv/Trigger "{}"
```

应看到 `gazebo_attachment_follower`、`/gazebo_grasp/attach`、`/gazebo_grasp/detach`。如果服务名存在但节点不存在，先停止残留进程并完整重启，不要继续判断抓取几何问题。

### 4. Gazebo 状态服务

```bash
ros2 service type /get_entity_state
gazebo_msgs/srv/GetEntityState
ros2 service type /set_entity_state
gazebo_msgs/srv/SetEntityState
ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: red_test_box, reference_frame: world}"
```

## 高度、碰撞和附着的判定原则

- `red_test_box` 的 collision 必须一直存在；不能用删除碰撞模型来掩盖夹爪把物体推走的问题。
- 物体中心、顶部高度、TCP、预抓取高度要分别核对；`object_height_m` 只表示尺寸，不等于桌面高度。
- 夹爪先接近到预抓取位姿，再低速下降和闭合；如果物体在闭合前已经产生明显位移，优先修正 TCP、指尖 collision、pin 几何和接近轨迹。
- attachment follower 是当前确定性 MVP：它在 attach 后保存物体相对 `grasp_tcp` 的位姿，并通过 `/set_entity_state` 跟随；它不是 Gazebo 原生固定关节插件。
- 运输和放置仍必须保持物体 collision，detach 后让物体重新参与物理碰撞并落入料箱。

## 当前保留但不作为活动入口的文件

为了保留回退和历史对照，新目录仍有旧 AG95 文件和旧配置，例如 `urdf/ag95_macro.xacro`、`urdf/cr7as_grasp_gazebo.urdf.xacro`、`launch/gazebo_control.launch.py`、`config/gazebo_controllers.yaml`。它们不会被 AG165 完整场景自动调用，也没有删除；不要将它们和 AG165 入口同时启动。

## 交付边界

Windows 端已完成目录整理、网格复制、XML 检查、路径检查和文档整理；没有伪造 Ubuntu 的 Gazebo/MoveIt 运行结果。Ubuntu 首次运行时如果失败，请按 `docs/Ubuntu迁移与验证.md`、`docs/AG165_SDF闭环说明.md` 和 `docs/交付清单与当前生效入口.md` 顺序排查。

后续任何代码、Xacro、YAML、launch 或模型修改，都要同步更新本 README 和 `docs/` 中对应的验证命令。



## 2026-10-16 稳定抓放 Demo：运动学附着模式

本工作区当前优先目标是**可重复演示视觉定位 → 接近 → 闭爪 → 搬运 → 深框放置**，而不是以 Gazebo 摩擦/接触数值证明真实夹持力。因此新增了 Demo 专用的运动学附着层，且明确与“纯物理夹持测试”分离。

- `red_test_box` 加载 `libdemo_attachment_physics_plugin.so`。收到 `/gazebo_grasp/set_demo_attachment_physics` 的 `SetBool(true)` 后，插件在 **Gazebo world-update/物理线程**关闭该工件 link 的重力和 collision response；`false` 时恢复默认 collision mask 与重力。
- `attachment_follower.py` 在 `/gazebo_grasp/attach` 成功时先记录工件相对 `grasp_tcp` 的即时位姿，再请求启用上述 Demo 物理模式，最后才开始 follower；工件在附着期间不再同时被 ODE 的重力、桌面或指尖 collision 推开。
- `/gazebo_grasp/detach` 先停止 follower、清零工件速度，再请求恢复重力/碰撞；随后执行器才打开夹爪和撤离。
- 该模式是**稳定流程演示机制，不是物理摩擦抓取证明**。`/vision_guided_motion/execute_physical_pick_lift_test` 仍不 attach，保留作以后真实夹持验证。
- `pick_place.yaml` 的 `demo_attachment_mode: true` 与 `demo_allow_close_goal_as_grasp: true` 仅作用于常规 Demo 服务：闭爪轨迹达到配置的 `gripper_close_position` 但没有严格双指接触候选时，状态机记录 `DEMO_GRASP_ASSUMED` 并允许进入运动学附着。`execute_contact_test` 和 `execute_physical_pick_lift_test` 不使用此放行。

同步并构建：

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  cr7as_grasp cr7as_vision_grasp cr7as_gazebo_grasp cr7as_vision_motion
source install/setup.bash
ros2 launch cr7as_vision_motion demo_pick_place.launch.py
```

启动后必须先确认模型插件服务存在：

```bash
ros2 service type /gazebo_grasp/set_demo_attachment_physics
# 期待：std_srvs/srv/SetBool
ros2 service list | grep -E 'gazebo_grasp/(attach|detach|set_demo_attachment_physics)'
```

完整 Demo 触发：

```bash
ros2 service call /vision_guided_motion/execute_pick_place std_srvs/srv/Trigger '{}'
```

验收：附着后状态话题出现 `attached`；红色工件稳定跟随 TCP、没有上下弹跳；放置后出现 `detached`，工件恢复物理并留在深框，夹爪张开后安全撤离。若插件服务不存在或 attach 返回 `Demo attachment refused`，说明 Ubuntu 仍运行旧模型/旧库：停止全部 Gazebo 进程，确认新 `model.sdf` 和 `libdemo_attachment_physics_plugin.so` 已同步并重新构建，再完整重启。

## 2026-10-16：料框一致坐标与半高 Demo 放置

当前流程演示将 `place_bin` 的 Gazebo 生成位置、MoveIt 碰撞模型中心和放置 TCP 的 XY 统一为 `base_link/world: [0.20, 0.50]`（当前 `base_link` 与 world 原点重合）。料框底板保持 `0.25 x 0.25 x 0.04 m`；四面墙由原先 `0.16 m` 高改为 `0.08 m` 高、厚 `0.01 m`，墙顶为 `z=0.74 m`。墙的位置同步按外轮廓 `0.25 m` 重新计算为 `±0.12 m`，Y 墙长度为 `0.23 m`，避免 Gazebo 与 MoveIt 碰撞盒不一致。

Demo 放置 TCP 使用 `[0.20, 0.50, 0.780]`，高于料框顶沿 40 mm；这不是让 AG165 深入料框的真实插入动作，而是稳定 Demo 的“箱口上方 detach/open，工件恢复重力后落入料框”策略。`place_clearance_pose_xyz` 为 `[0.20, 0.50, 0.860]`。修改以下三个文件中任意一个坐标/尺寸时，必须同步其它两个，并完全重启 Gazebo/MoveIt：

- `src/cr7as_vision_grasp/launch/spawn_grasp_scene.launch.py`：Gazebo 生成坐标；
- `src/cr7as_vision_grasp/models/place_bin/model.sdf`：视觉和 Gazebo 碰撞几何；
- `src/cr7as_vision_motion/config/pick_place.yaml`：MoveIt 碰撞几何与放置目标。

## 2026-10-16：整体桌面场景下移 150 mm（受控高度试验）

为区分“放置下降被料框碰撞拒绝”和“末端在当前高度不可达”，当前 Demo 将**桌面、红色工件和料框整体下移 0.15 m**，而不改变机器人 base_link。数值统一为：桌面 spawn/MoveIt 碰撞中心 `z=0.45`（桌面顶面 `z=0.47`）；P2 投影平面 `plane_z_m=0.47`；红块初始中心 `z=0.50`；料框模型原点 `z=0.47`；料框顶沿 `z=0.59`；Demo 释放 TCP 为 `z=0.63`，clearance TCP 为 `z=0.71`。必须将 `spawn_grasp_scene.launch.py`、`demo_pick_place.launch.py`、`pixel_to_table_pose.yaml`、`pick_place.yaml` 和执行器中 MoveIt table collision 的高度一起更新并完整重启。此改动只用于排查高度/碰撞裕量，不构成真实产线标定。

## 2026-10-16：500 × 500 mm 料框与桌面边界约束

Demo 料框底板已扩大为 `0.50 x 0.50 x 0.04 m`。墙厚保持 `0.01 m`、墙高保持 `0.08 m`：X 墙为 `0.01 x 0.50 x 0.08 m`，局部中心 `x=±0.245 m`；Y 墙为 `0.48 x 0.01 x 0.08 m`，局部中心 `y=±0.245 m`。料框中心统一为 `[0.20, 0.70, 0.47]`。桌面为 `1.20 x 0.80 m`、中心 `[0.00, 0.80]`，其台面范围为 `x=[-0.60, 0.60]`、`y=[0.40, 1.20]`；新料框外轮廓为 `x=[-0.05, 0.45]`、`y=[0.45, 0.95]`，因此四面均在桌面范围内，Y 负侧仍保留 50 mm 台面余量。Gazebo spawn、MoveIt 碰撞模型和放置目标均已同步为该中心；改尺寸或中心时必须同时维护三处。

## 2026-10-16：料框改为 380 × 380 mm，并向可达区域内移

料框底板由 `0.50 x 0.50 m` 改为 `0.38 x 0.38 m`，墙厚仍为 `0.01 m`、墙高仍为 `0.08 m`。X 墙尺寸为 `0.01 x 0.38 x 0.08 m`、局部中心 `x=±0.185 m`；Y 墙尺寸为 `0.36 x 0.01 x 0.08 m`、局部中心 `y=±0.185 m`。料框世界/`base_link` 中心统一改为 `[0.10, 0.60, 0.47]`，同时放置 clearance/release 的 XY 也改为 `[0.10, 0.60]`。

桌面平面范围为 `x=[-0.60, 0.60]`、`y=[0.40, 1.20]`；新料框外轮廓为 `x=[-0.09, 0.29]`、`y=[0.41, 0.79]`，没有超出桌面，且比先前 `[0.20,0.70]` 的中心更接近机械臂。`model.sdf`、Gazebo spawn、MoveIt YAML 三处已经同步；任何后续尺寸或中心调整必须同步三处。

## 四层统一标准（当前工程语义基线）

视觉抓取、AG165 闭环、Gazebo 与 MoveIt 的坐标、几何、状态和模式约定已统一记录在：[`docs/CR7AS_AG165_四层统一标准_v1.1.md`](docs/CR7AS_AG165_四层统一标准_v1.1.md)。深框放置、料框尺寸/位置、attachment、物理抓取验证的任何后续改动均须遵循该文档，并同步相关 README。


> **2026-10-16：** 深框放置实现以 docs/CR7AS_AG165_四层统一标准_v1.1.md 为当前语义基线；v1.0 仅用于历史追溯。

## 2026-10-16：附着几何同源基线（进行中）

为消除 Gazebo follower 与 MoveIt 对被抓工件相对 `grasp_tcp` 位姿不一致的问题，新增 `/gazebo_grasp/attachment_snapshot`（`geometry_msgs/msg/PoseStamped`，`frame_id=grasp_tcp`，transient-local）。follower 在 attach 成功瞬间发布唯一的 `T_grasp_tcp_object`；执行器只消费**本轮 attach 后的新 snapshot**，并将其原样写入 MoveIt `AttachedCollisionObject`。如果 snapshot 未在 `attachment_snapshot_timeout_sec=2.0` 秒内到达，则进入 `SAFE_STOP_ATTACHED`，不再创建旧的零偏移附着物。

本轮是四层标准的第 1 步，未开启 Deep-Bin 插入。完整数据采集与 Ubuntu 命令见 [`docs/2026-10-16_附着几何同源_第一阶段验证.md`](docs/2026-10-16_附着几何同源_第一阶段验证.md)。

## 2026-10-16：Lift 起始桌面接触的局部碰撞豁免

实测的 `T_grasp_tcp_object` 使 MoveIt 正确看到红块底面与桌面相切。为避免 `Fix Start State In Collision` 将真实状态替换为远距离 jiggle state，`pick_place_executor` 新增 `allow_object_table_contact_during_lift: true`。它只在 **Lift 规划和执行期间** 临时允许 `red_test_box ↔ table`；Lift 后立即恢复。料框、夹爪、桌面及其余碰撞对从未关闭。详细原理和验收命令见 [`docs/2026-10-16_附着几何同源_第一阶段验证.md`](docs/2026-10-16_附着几何同源_第一阶段验证.md)。

## 2026-10-16：Demo 微抬升诊断和直线路径

`execute_pick_lift_test` 现在在夹爪 Hold 后发布 `AG165_SETTLING_DIAGNOSTIC`，记录 0.50 秒内六个 AG165 闭环关节的位置范围与最终左右差；它只用于采集阈值，不能拒绝当前 Demo 附着，也不能证明真实物理夹持。随后 30 mm Lift 使用 `computeCartesianPath()` 的世界 `+Z` 直线（2 mm 插补、fraction >= 0.99），替代自由 OMPL 关节空间规划。完整说明见 [`src/cr7as_vision_motion/README.md`](src/cr7as_vision_motion/README.md) 和 [`docs/2026-10-16_附着几何同源_第一阶段验证.md`](docs/2026-10-16_附着几何同源_第一阶段验证.md)。

## 2026-10-16：Deep-Bin 空夹爪插入预检

在 3/3 Demo 笛卡尔微抬升通过后，下一步不是直接调用完整放置，而是调用 `/vision_guided_motion/execute_deepbin_dry_run`：空夹爪从 `place_clearance_pose_xyz` 规划到箱口上方，再以 world `-Z` 的 `computeCartesianPath()` 插入至 `place_pose_xyz`，随后 world `+Z` 撤离。它从不附着/释放工件，且不允许任何 gripper/bin 碰撞豁免。详见 [`src/cr7as_vision_motion/README.md`](src/cr7as_vision_motion/README.md)。

### Deep-Bin 空载预检失败状态

`DEEPBIN_DRY_RUN_INSERT_REJECTED` 表示未执行下潜；`SAFE_STOP_DEEPBIN_INSERT` 表示执行阶段异常且机器人可能部分移动，必须检查并重启场景。二者都没有 attach、detach 或释放工件。

### Deep-Bin 预检闭合包络

首个 dry-run 的 `fraction=0.725` 是全开 `q=0.0` AG165 与料框的碰撞结果，不是闭合持件状态不可插入的结论。预检已改为 `deepbin_dry_run_gripper_position: 0.35`：闭合/持件状态插入，partial release 在后续单独验证，永不全开进入深框。

### Deep-Bin Top-Down 候选预检

实测 clearance 位姿下 `grasp_tcp +Z≈base +Y`，说明旧 `Rx(-90°)` 是水平指端轴，且正是 `fraction=0.725` 的主要几何原因。空夹爪预检现使用独立的 `Rx(180°)` top-down 基准（`grasp_tcp +Z -> base -Z`）及 `[0,90,180,270]` yaw 搜索；不改 `grasp_tcp`、抓取姿态或全局 `place_orientation`。通过仅证明闭合夹爪本体的深框进出几何，带工件仍须后续验证。

## 2026-10-17：Deep-Bin 白盒候选诊断（Phase 0 / Phase 1）

`pick_place_executor` 现在和 `move_group` 一样显式加载 `robot_description`、`robot_description_semantic` 与 `robot_description_kinematics`。这消除了执行器自己的 `RobotModelLoader` 缺少 `kinematics.yaml` 的配置缺口；`src/cr7as_grasp_moveit_config/config/kinematics.yaml` 中的 `ARM -> KDLKinematicsPlugin` 成为同一份运动学来源。

新增只读服务：

```text
/vision_guided_motion/probe_deepbin_candidates
```

它**不会**下发夹爪指令、附着工件、调用 OMPL、执行轨迹或移动机器人。针对 `place_clearance_pose_xyz` 的固定 XY，以 `deepbin_candidate_probe_clearance_z_m` 和 `deepbin_candidate_probe_yaw_degrees` 组合生成 Top-Down 候选；每一个候选严格按以下顺序诊断：

```text
ComputeIK(avoid_collisions=false)
  -> IK_FAIL / IK_TIMEOUT
  -> CheckStateValidity(返回的关节解)
       -> COLLISION [link_a <-> link_b]
       -> VALID
```

因此它将原先笼统的 `Unable to sample any valid states for goal tree` 拆分为纯 IK 不可达、服务/状态问题或具体碰撞对。首轮参数为中心 XY、Z=`[0.710, 0.760, 0.810, 0.860] m`、Yaw=`[0,90] deg`；仅在此诊断结果明确后才修改高度、料框候选 XY 或进入一次 OMPL/Cartesian dry-run。

构建并完整重启场景后运行：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/dev_ws_sdf_gripper/install/setup.bash
ros2 service list | grep -E 'compute_ik|check_state_validity|probe_deepbin_candidates'
ros2 topic echo /vision_guided_motion/pick_place_status
# 新开终端：
ros2 service call /vision_guided_motion/probe_deepbin_candidates std_srvs/srv/Trigger '{}'
```

成功仅表示“探测服务完成”；最终消息中的 `valid=N` 才表示存在 `N` 个可达且无碰撞的 clearance 候选。该服务不构成深框插入或抓放成功。

## 2026-10-17：Phase 1.1 水平对照与手腕分支

`probe_deepbin_candidates` 先运行 `DEEPBIN_CANDIDATE_CONTROL_HORIZONTAL`：它使用当前 `place_clearance_pose_xyz` 与历史水平 `place_orientation_xyzw`，但仍通过完全相同的 `/compute_ik -> /check_state_validity` 服务链。该对照组不移动机器人，也不证明深框可插入；它只验证 IK API、`grasp_tcp`、基准坐标和运动学插件可共同得到已知姿态的解。

随后才运行 Top-Down 候选。`Rx(180°)` 与 `Ry(180°)` 均令 TCP +Z 指向 base −Z，且 `Ry(180°)=Rz(180°)*Rx(180°)`；因此工程只保留一个 Top-Down 基准四元数 `[1,0,0,0]`，并以 Yaw `[0,90,180,270]` 覆盖手腕构型分支。虽然 0/180 的夹爪外包络近似相同，CR7AS 的 J4/J5/J6 限位与奇异性不必对称，不能在 IK 层去重。

报告中的 `horizontal_control_valid=1` 且 Top-Down 仍全为 `IK_FAIL(-31)`，才可以严谨地归因于“当前料框位置的 Top-Down 构型不可达”；若水平对照也失败，则先处理运动学配置、目标 link 或坐标链，不得继续扩大 XY/Z 搜索。

## 2026-10-17：Phase 1.2 料框内受控 XY 网格

Phase 1.1 已证明水平对照 `VALID`，但料框中心 `[0.10,0.60]` 的 16 个 Top-Down（四个高度 × 四个 yaw）全部 `NO_IK_SOLUTION (-31)`。Phase 1.2 不修改料框、碰撞体或机器人控制，而是固定 entry-plane `z=0.710 m`，以 `place_clearance_pose_xyz` 为中心应用 `x/y=[-0.12,0,+0.12] m` 偏移：得到 `X=[-0.02,0.10,0.22]`、`Y=[0.48,0.60,0.72]` 的 3×3 网格。每个 XY 再测 Top-Down yaw `[0,90,180,270]`，共 36 个候选，加一个水平对照共 37 个。

仍然仅调用 `compute_ik(avoid_collisions=false)` 和 `check_state_validity`：不会闭爪、附着、OMPL 规划、执行轨迹或改变 Gazebo。若此网格仍没有任何 `VALID`，可严谨判定当前 380 mm 料框覆盖区域不存在该 CR7AS/AG165 配置的 Top-Down entry pose；下一步应调整深框工艺姿态或料框工作站位置，而不是继续尝试 Cartesian 插入。

## 2026-10-17：Phase 1.3 全局 Top-Down TCP 工作空间粗图

新增只读服务：

```text
/vision_guided_motion/probe_topdown_workspace_map
```

它以 `base_link` 为参考，在 `x=[-0.40,0.40] m`、`y=[0.20,0.90] m`、`z=0.710 m` 的 50 mm 粗网格上，针对 yaw `[0,90,180,270]` 建立 Top-Down **TCP/夹爪闭合工具包络**工作空间图。默认总计 `17×15×4=1020` 个候选。每个候选先执行无碰撞 IK，再对返回解调用场景有效性检查；其输入 RobotState 强制写入六个 AG165 关节 `q=0.35`，不依赖 Gazebo 当前夹爪状态，也绝不向 Gazebo 发送夹爪或机器人命令。

状态话题每行 `TOPDOWN_WORKSPACE_MAP_ROW` 的 `cells` 对应从 `x_min` 开始、按 `step` 递增的 X 栅格；图例为 `V=IK 且无碰撞`、`I=无 IK`、`C=碰撞`、`T=服务超时`。这是 `W_free_tcp` 的粗采样，**不是**红块中心可放置集合。后续必须使用实测 `T_grasp_tcp_object` 将 `B_object` 变换为各 yaw 下的 `B_tcp(yaw)` 后，才可进行 `W_free_tcp ∩ B_tcp(yaw)` 的真实放置候选求交。

运行此服务时可能需要几十秒，因为当前通过 `/compute_ik` 和 `/check_state_validity` ROS 服务顺序请求；不应把它误称为纯内存毫秒级扫描。

## 2026-10-17：工作台落地与场景 Z 基线（当前生效）

`models/table/model.sdf` 的台面为 `1.20 × 0.80 × 0.04 m`，四条桌腿中心为模型局部 `z=-0.095 m`、高度 `0.150 m`。从桌腿底部至台面顶面的总高度为 **0.190 m**（桌腿 `0.150 m` + 台面 `0.040 m`）。

当前已统一校正：table spawn 及 MoveIt 台面碰撞中心为 `z=0.170 m`，四条腿 visual/collision 下表面为 world `z=0.000 m`，台面顶面为 `z=0.190 m`。料框底板下表面（`place_bin` origin）同步为 `z=0.190 m`，红块中心同步为 `z=0.220 m`，P2 `plane_z_m=0.190`。相对场景高度不变的 place/entry 目标也下降 `0.230 m`：`place_pose_xyz.z=0.350`、`place_clearance_pose_xyz.z=0.430`。完整重启前不得混用新旧高度。


## 2026-10-17：双支撑台场景（当前生效）

大桌子模型仍保留作历史资源，但 `spawn_grasp_scene.launch.py` 已不再生成它。当前场景使用两个静态方形支撑台，且 Gazebo 与 MoveIt 碰撞几何同源：

| 实体 | 中心 XY | 支撑体尺寸 (m) | 支撑顶面 Z (m) | 被支撑实体 |
|---|---|---|---:|---|
| `object_support` | `[-0.20, 0.50]` | `0.14 × 0.14 × 0.47` | `0.470` | `red_test_box`，中心 Z=`0.500` |
| `bin_support` | `[0.10, 0.60]` | `0.42 × 0.42 × 0.20` | `0.200` | `place_bin`，模型 origin Z=`0.200` |

P2 的 `plane_z_m=0.470` 是红块支撑上表面，不是料框高度。放置 TCP 保持相对于料框不变：place Z=`0.360`、clearance Z=`0.440`。Lift 时临时 ACM 放行对从 `red_test_box <-> table` 改为唯一的 `red_test_box <-> object_support`，完成抬升后立即恢复。必须同步模型、launch、P2 配置、执行器 YAML/C++ 后完全重启 Gazebo 和 MoveIt。

### 2026-10-17：视觉复核拒绝的可解释诊断

`Visual target moved or became invalid before final grasp` 是保护性总文案；此前无法区分无视觉消息、`metric_pose_valid=false`、置信度不足、姿态消息过期或与预抓取前坐标偏差超阈值。执行器现在会在该总文案之前发布 `VISUAL_TARGET_REVALIDATION_FAILED`，并精确给出失败项，或输出 `current/reference/delta/norm/tolerance/confidence/age_sec`。该日志改动不放宽门禁、不改变抓取、附着或轨迹；须先依赖该证据定位相机被机械臂遮挡、检测置信度不足或视觉坐标跳变。

## 2026-10-17：Phase 1.2 大桌面基线恢复

为隔离“深框高度调整”与“整套场景换支撑台”的变量，当前 Demo 恢复为单一大桌面基线。Gazebo `table` spawn 为 `[0.00, 0.80, 0.450]`，桌面顶面为 `z=0.470 m`；红块仍为 `[-0.20,0.50,0.500]`，底面恰接触桌面；`place_bin` origin 恢复为 `[0.10,0.60,0.470]`。MoveIt 使用同一张 `1.20×0.80×0.04 m` table collision box，并且 Lift 期间仅临时允许 `red_test_box <-> table`。料框的 place pose / clearance 分别为 Z=`0.630/0.710 m`，Phase 1.2 候选扫描 entry plane 为 `z=0.710 m`。`object_support` 与 `bin_support` 资源保留在包内以便日后单独实验，但本基线 launch 不生成它们。

## 2026-10-17：分体台面（红块侧保持高度，料框侧降低 350 mm）

当前场景将原 `1.20×0.80 m` 台面沿 world/base_link X 方向在 `x=-0.070 m` 切为两个静态台面。红块中心为 `x=-0.200 m`、尺寸 `0.060 m`，其 +X 边缘为 `-0.170 m`；切面因此在工件 +X 边缘外留出 `0.100 m`。`pick_table` 覆盖 X=`[-0.600,-0.070]`、Y=`[0.400,1.200]`，顶面 Z=`0.470 m`，红块仍为 `[-0.20,0.50,0.500]`；`bin_table` 覆盖 X=`[-0.070,0.600]`、相同 Y 范围，顶面 Z=`0.120 m`，恰好低 `0.350 m`。料框 origin 改为 `[0.10,0.60,0.120]`，其 place/clearance TCP Z 同步为 `0.280/0.360 m`；P2 `plane_z_m=0.470` 保持不变，因为红块支撑面未变。MoveIt 使用独立的 `pick_table`、`bin_table` collision boxes；Lift 只临时允许 `red_test_box <-> pick_table`。 

## 2026-10-17：Phase 2 空夹爪深框插入 Dry-Run

分体台面后的 Phase 1.2 probe 在 Z=`0.360 m` 得到 10 个有效 Top-Down endpoint。首个受控执行候选固定为 `deepbin_dry_run_target_xy=[0.10,0.48]`、yaw=`90°`；这是料框 X 中心、近侧安全行，不修改生产名义 `place_pose_xyz=[0.10,0.60,0.280]`。`/vision_guided_motion/execute_deepbin_dry_run` 读取该候选后，以空夹爪 `q=0.35` 规划到 clearance Z=`0.360`，再以 2 mm 步长沿 world -Z 插入至 Z=`0.280` 并原路 +Z 退回。它不读取视觉目标、不附着、不搬运红块、不松爪。通过只证明该空夹爪候选的进退轨迹；后续真实放置仍须重新计算附着物/TCP 几何与释放包络。

## 2026-10-17：深框浅插入验证边界

固定 `XY=[0.10,0.48]`、yaw=`90°` 的 endpoint probe 已证明 Z=`0.360..0.320 m` 有效，而 `0.310 m` 无 IK 解、`0.300 m` 与 `wall_y_min` 碰撞。较低处重新出现的孤立 IK 解不代表连续下潜可行。Deep-Bin Dry-Run 因此改为独立验证 `0.360 -> 0.320 m` 的空夹爪直线插入；它不附着、不释放，也不是最终生产放置深度。

## 2026-10-17：深框倾角入口预检

最终深框工艺不能由 40 mm 浅插入替代。执行器增加不执行运动的倾角 endpoint probe：在保持 TCP XY/Z 的条件下，以 TCP 局部水平轴的 0/90/180/270°方向尝试 5°、10°等小倾角，并将结果区分为无 IK、环境碰撞或有效 endpoint。probe 统一虚拟夹爪开度为 q=0.35，使碰撞包络不依赖运行瞬间的 Gazebo 反馈。通过 endpoint 预检后，仍必须单独验证实际连续的倾斜插入轨迹，才可讨论附着红块、底部接近与受限开爪。
