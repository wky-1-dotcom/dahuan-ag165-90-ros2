# AG165 SDF 闭环：架构、坐标与故障复盘学习指南（v2.1）

> **适用工程**：CR7AS + AG165、ROS 2 Humble、Gazebo Classic、MoveIt 2。  
> **文档性质**：学习、排障与设计边界文档；不是深框任务的放行依据。深框任务的放行仍需以当前场景、PlanningScene、实际夹爪开度和运行日志为准。  
> **源码核验基线**：本文于 2026-10-19 依据当前工程源码整理。代码块是“关键执行片段”，不是把 CAD 网格、惯量和全部样板代码复制一遍。

## v2.1 更新说明：采纳内容与源码纠偏

本版吸收了 v2.1 全链路复盘中的“自由度、历史故障、SDF pose 语义、双模式参数、验证命令和真机迁移”内容，但以下项目以**当前源码**为准，而没有照抄历史笔记：

| 项目 | v2.1 当前有效结论 |
|---|---|
| 运行时 close 目标 | `pick_place.yaml` 覆盖后为 `0.35 rad`；`pick_place_executor.cpp` 中的 `0.62 rad` 是 C++ 参数默认值。二者不能混称“唯一标准闭合值”。 |
| soft mimic 参数 | 当前 Xacro：`kp=2.0`、`kd=0.35`、`effort=2.0`；不是历史表格中的 `1.5/0.3/1.5`。 |
| Demo physics plugin | 只用于 Demo 附着物理隔离。它不启动/停止 follower，也不证明物理夹持。 |
| 真机状态码 | 未依据当前 AG165 固件通信手册核验，不把任意 `0/1/2/3` 数值写成接口事实。 |
| SDF 参考模型 | 当前工程可核验的独立对照文件为 `models/ag165_90_4/model.sdf`。 |

---

## 0. 先给结论：这套系统实际在做什么

AG165 每侧是四连杆闭环。URDF/MoveIt 只能可靠处理无环的关节树；Gazebo 则需要额外的物理销轴使机构闭合。因此本工程不是“URDF mimic 夹爪”，而是下面的双层模型：

```text
ROS / MoveIt 运动学树                         Gazebo 物理闭环
────────────────────                         ───────────────
joint1_left  (唯一 ROS 命令关节)  ──PID──►   左侧四连杆 + SDF pin
joint1_right (只读状态)            ◄─PD──    mimic 插件跟随左侧主动关节
joint2/3_left/right (只读状态)      ◄────    ODE 解算的被动连杆

             grasp_tcp（固定工具参考系）
                         │
                 Demo follower（仅演示）
                         │
               red_test_box 的运动学随动
```

**最重要的边界**：

1. SDF pin 只服务 Gazebo 物理闭环，绝不可进入 MoveIt 规划组或 JointTrajectoryController。
2. `grasp_tcp` 是固定在 `gripper_base` 上的工具参考系；它不是手指、不是接触点、也不会因开合而移动。
3. `attachment_follower.py` + Demo physics plugin 是确定性演示附着，不能证明摩擦、夹持力或真机抓稳。
4. MoveIt 规划用的碰撞开度必须与 Gazebo 实际开度同源；不能只给 `joint1_left` 一个数，却把其余五个关节留在旧状态。

---

## 1. 机构与控制职责

### 1.1 每侧闭环为何不能直接写成 URDF 环

```text
ag165_base_link -- joint1_left（主动） --> link1_left -- joint2_left（被动） --> link2_left
       │                                                                          │
       └──────── joint3_left（被动） -------------------------------> link3_left ──┘
                                     Gazebo-only SDF pin: link2_left ↔ link3_left
```

若将最后一条 `link2 ↔ link3` 也放入 URDF，某个 link 会出现两个父连接，TF/MoveIt 的树结构即失效。故采用：

| 模型层 | 表达内容 | 用途 |
|---|---|---|
| URDF/Xacro | 六个树关节、网格、碰撞体、`grasp_tcp` | TF、FK/IK、MoveIt、ros2_control 接口 |
| Gazebo 扩展 SDF | 两条 `link2 ↔ link3` revolute pin | ODE 物理约束、闭环几何 |
| mimic ModelPlugin | `joint1_right` 对 `joint1_left` 的 PD 跟随 | 双侧同步输入 |
| ros2_control | 仅给左主动关节位置命令 | 轨迹 action 与状态发布 |

### 1.2 当前六个树关节的真实角色

| 关节 | 角色 | 命令来源 | 控制器中是否可命令 |
|---|---|---|---|
| `ag165_joint1_left` | 左主动输入 | `gripper_controller` → Gazebo position PID | 是，唯一一个 |
| `ag165_joint1_right` | 右侧镜像输入 | `Ag165GripperMimicPlugin` 的 PD 力矩 | 否，只读 |
| `ag165_joint2_left/right` | 被动副 | ODE + SDF pin | 否，只读 |
| `ag165_joint3_left/right` | 被动副 | ODE + SDF pin | 否，只读 |

因此，`joint_states` 有六个 AG165 状态是**正确设计**；但轨迹命令只包含 `ag165_joint1_left` 也是**正确设计**。

---

## 2. 坐标系、TCP 与右手定则

### 2.1 必须区分的坐标系

| 坐标系/对象 | 含义 | 不能混淆为 |
|---|---|---|
| `world` | Gazebo 世界坐标，包含重力与物体实体位置 | 机器人规划根坐标的必然同义词 |
| `base_link` | CR7AS 运动学根坐标，MoveIt 通常在此规划 | Gazebo 世界原点，除非 TF 明确重合 |
| `l6` | 机器人末端法兰上游 link | 夹爪指尖 |
| `gripper_base` | 固定安装在 `l6` 的 AG165 基座 | TCP |
| `grasp_tcp` | 固定工具参考系，抓取/附着/目标位姿的公共基准 | 可变手指接触点 |
| `red_test_box` | Gazebo 中的被抓物模型 | MoveIt AttachedCollisionObject 自动同义物 |

当前 `grasp_tcp` 的固定变换来自 `gripper_base`，偏移为 `xyz="0 0.00071813 0.18132"`、`rpy="0 0 0"`。这只是**当前模型定义**；真机要通过工具标定建立真实法兰到 TCP 的变换。

### 2.2 右手定则怎么用于关节轴

对任意关节轴向量 `+axis`：右手拇指指向轴正方向，四指弯曲方向就是正关节角方向。不要根据屏幕上的“左/右手指看起来开/合”猜正负号；应从：

1. 关节 `<axis xyz="...">`；
2. 父坐标系中的轴方向；
3. 小正增量 `q → q + 0.01` 的 Gazebo 实际运动；
4. 两侧镜像后的相对开合；

共同判定。

当前左右 `joint1` 的 URDF 角度正方向并不要求视觉上互为负号。此模型验证后使用 mimic `multiplier=+1.0`；改为 `-1` 会使右侧朝错误的物理方向驱动，不能只凭“名称是右侧”强行取负。

### 2.3 SDF `relative_to` 是闭环销轴的坐标契约

下列 pin pose 的数值 `0.022 0 0` 是在 **link2 局部系**下表达，而不是 `world` 或 `link3` 下表达：

```xml
<pose relative_to="ag165_link2_left">0.022 0 0 0 0 0</pose>
<parent>ag165_link2_left</parent>
<child>ag165_link3_left</child>
```

省略 `relative_to` 时，SDF joint pose 的默认语义不是这里所需的 `link2` 局部系。结果通常是销轴被放错位置：可视网格似乎还在，物理约束却会拉扯、重叠、震荡或单侧卡死。

---

## 3. 运行链路：从轨迹目标到物理手指

```text
pick_place_executor
  │ FollowJointTrajectory(goal: ag165_joint1_left=q)
  ▼
gripper_controller / JointTrajectoryController
  ▼
GazeboGripperSystem：position interface + pos_kp/ki/kd
  ▼
Gazebo 的 joint1_left 物理力矩
  ├─► 左侧 ODE + SDF pin 解算被动连杆
  └─► mimic plugin 读取 joint1_left
          │ target = offset + multiplier × active_position
          ▼
       对 joint1_right 施加限幅 PD 力矩
          ▼
       右侧 ODE + SDF pin 解算被动连杆
```

### 3.1 现行控制器契约

`gripper_controller` 只有左主动关节；`open_loop_control: false` 使 action 使用反馈状态判断结果。`goal_time: 5.0` 解释了“关节实际被接触卡住但仍试图到固定位置”时，最终可能报 `goal_time_tolerance` 超时。

```yaml
gripper_controller:
  ros__parameters:
    joints: [ag165_joint1_left]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_partial_joints_goal: false
    open_loop_control: false
    constraints:
      goal_time: 5.0
      stopped_velocity_tolerance: 0.05
      ag165_joint1_left:
        trajectory: 0.30
        goal: 0.03
```

**不要把关节角直接当成毫米开口。** 当前主动关节限位为 `[-0.05, 0.93] rad`，但“开/合的语义、可夹工件宽度、受限释放开度”必须通过 Gazebo 测量或真机标定表确定。过去试验中的 `q=0.35` 只是某个 dry-run 的包络测试值，并非 AG165 的通用“闭合值”。

### 3.2 参数优先级：为何 `0.35` 与 `0.62` 同时出现

当前工程中两处数值具有不同优先级：

```text
pick_place_executor.cpp: declare_parameter("gripper_close_position", 0.62)
                                   │ 仅当没有 YAML 覆盖时才生效
                                   ▼
pick_place.yaml: gripper_close_position: 0.35
                                   │ 当前 launch 加载此 YAML 时的有效运行值
                                   ▼
执行器实际 close 目标 = 0.35 rad
```

所以：

- `0.0 rad` 是当前 YAML 的 open 目标；
- `0.35 rad` 是当前场景配置的 close 目标；
- `0.62 rad` 是代码默认值和历史空载测试候选，不能在不改 YAML 的情况下声称它正在运行；
- partial release 也必须作为显式配置，并以“工件已脱离指腹、外包络仍能退出料框”为准通过标定，而不是固化一个放之四海皆准的角度范围。

### 3.3 四连杆自由度与“为何施力也推不动”

单侧平面四连杆有四个构件（含基座）和四个单自由度转动副。按 Grübler 公式：

\[
F=3(n-1)-2j=3(4-1)-2(4)=1
\]

它只有一个独立自由度。`joint1_left` 受力后，`joint2_left`、`joint3_left`、SDF pin 必须被动配合。若给被动关节也添加 position command、硬 mimic 或不恰当的保持约束，相当于再施加不相容的运动学方程，可能令有效自由度趋近 0：

```text
正确：主动轴受控 + 三个被动铰链自由 + pin 闭合  → F=1，可动
错误：主动轴受控 + 被动轴也强制位置 + pin 闭合  → 过约束，可能锁死
```

这里的“锁死”并非说明所有 Gazebo `hold_joints` 都必然错误。当前 `hold_joints=true` 与被动关节无 command interface 的组合是可行的；错误是把被动物理副也交给轨迹/位置控制器。

### 3.4 soft / reference profile：当前代码值

| 配置 | `ag165_soft_mimic_test=true` | `false` | 解释 |
|---|---:|---:|---|
| right `multiplier` | `+1.0` | `+1.0` | 两种模式均不应改成 `-1.0` |
| mimic `kp / kd / effort` | `2.0 / 0.35 / 2.0` | `2.5 / 0.35 / 2.5` | 由 Xacro 写入 ModelPlugin |
| `joint1_right` damping | `0.15` | `0.20` | 右侧轴阻尼 |
| `joint1_left` damping | `0.08` | `0.08` | 左侧轴阻尼 |
| `implicitSpringDamper` | true | 不额外启用 | 仅 soft profile 明确添加 |
| pin CFM / ERP | 左 `1e-4 / 0.2`，右 `1e-3 / 0.2` | 相同 | 当前 pin 约束参数 |

CFM 与 ERP 是 ODE 数值稳定性参数，不是“越硬越精确”的单向旋钮。调参顺序必须是：**插件加载 → 坐标系与销轴几何 → 被动关节无额外命令 → 主轴确有 PID 力矩 → 最后单变量调 PD/阻尼/CFM。**

---

## 4. 已踩过的故障：现象、根因、修复与预防

| 现象 | 经核验根因 | 已采取/应采取修复 | 长期预防 |
|---|---|---|---|
| 左主动关节收到 position 命令但机构不动 | 物理闭环不能只依赖裸 position interface；需要 Gazebo PID 输出力矩 | `GazeboGripperSystem` 为左主动关节配置 `pos_kp=2.5, pos_ki=0, pos_kd=0.35` | 每次改 URDF 后用单关节开合 smoke test 验证 |
| 机械臂保持不了姿态或似乎失去力矩 | 将机械臂 direct position 控制与夹爪 PID 混在一个 GazeboSystem，控制模式冲突 | 分离为 `GazeboArmSystem` 与 `GazeboGripperSystem`；开启 `hold_joints=true` | 不将不同 Gazebo 控制模式混进同一 hardware component |
| 右手指不动、反向或不同步 | 右主动关节不是 controller 命令对象；mimic 的 name/sign/offset/PD 参数不正确 | 由 ModelPlugin PD 跟随，当前模型使用 `multiplier=+1.0` | 用 `mimic_status` 检查 active/follower/target/force |
| link2/link3 重叠、抖动或销轴拉飞 | SDF pin pose 未在 link2 局部系定义，或闭环参数/碰撞体改变 | 显式 `relative_to="...link2..."`；保留已验证 pin 参数 | 修改任一销轴 pose/axis 后先禁用控制器做被动物理检查 |
| 夹到物体后抬升/规划失败 | 工件仍既是世界碰撞物又是附着物，或 table 支撑接触未做时序处理 | 附着使用实测 `T_grasp_tcp_object`；必要时短窗口 ACM 管理并在抬升后恢复 | 记录 attach snapshot；不要用零偏置替代实际相对位姿 |
| Demo 中物件稳定跟随但误认为物理抓稳 | follower 每周期调用 `/set_entity_state`，Demo plugin 同时关重力/碰撞 | 明确标为 Demo；真实验证须关闭该机制 | Physical 与 Demo 状态机和验收标准分离 |

### 4.1 不能再用的“修复方式”

- 给所有六个关节都下 trajectory 命令：会与闭环物理求解器争夺自由度。
- 将 SDF pin 放进 SRDF/MoveIt group：MoveIt 要的是树，闭环只属于 Gazebo。
- 为避免接触超时而无限增加 `goal_time`：这只会掩盖接触、包络或控制问题。
- 为了 Demo 稳定而把 attachment follower 的结果视作摩擦抓取成功。
- 使用当前 Gazebo 调出的 PID/摩擦数值直接下发真机。

---

## 5. 深框与夹爪：几何闭环要求

深框任务中，碰撞检查不是“TCP 点是否进入料框”，而是下列状态的完整几何：

```text
RobotState = ARM 6 joints
           + AG165 six tree-joint states（来自同一开度配置）
           + [抓取后] AttachedCollisionObject(T_grasp_tcp_object)
           + bin / table / remaining objects / Octomap
```

### 5.1 至少需要三种开度包络

| 阶段 | 规划场景必须使用的包络 |
|---|---|
| 进框预抓 | 预张开夹爪；包含指尖外侧与定位误差余量 |
| 持件进/退框 | 夹持开度 + 已附着工件的真实 `T_grasp_tcp_object` |
| 放置后退刀 | partial-release 开度；工件成为 world object 后还要按局部 ACM 窗口处理 |

“预抓开度 = 工件夹持方向投影宽度 + 两侧工艺余量”是几何规则；余量必须来自相机误差、夹爪标定和工件形状，不是固定写 `+20 mm`。

### 5.2 Demo attach 的正确理解

Demo attach 顺序是：读取 `world → object`，查询 `grasp_tcp ← world`，计算并发布 `T_grasp_tcp_object`，再使 follower 按该相对变换更新 Gazebo 物件位姿。附着物碰撞模型也必须使用同一份 snapshot。

它解决的是 **MoveIt 的附着几何与 Gazebo 演示位置一致**，不是实际指腹力、摩擦或工件滑移模型。

---

## 6. 可复现实验 SOP（夹爪本体）

### Gate A：模型和插件加载

1. 修改插件或 Xacro 后，先构建 `cr7as_grasp`；确认 mimic 动态库存在。
2. 启动整机 Gazebo launch，确认不额外 spawn 独立 `model.sdf` 夹爪。
3. 确认 `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 均 active。
4. 调用 mimic 状态服务；必须看到 active/follower 都 resolved 且 `enabled=true`。

```bash
cd ~/Desktop/dev_ws_sdf_gripper
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr7as_grasp
source install/setup.bash
ls -l install/cr7as_grasp/lib/libag165_gripper_mimic_plugin.so

ros2 service call /cr7as_ag165/mimic_status std_srvs/srv/Trigger '{}'
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

> 模型名若不是 `cr7as_ag165`，mimic 服务前缀也会随模型名改变；先 `ros2 service list | grep mimic`，不要硬猜服务名。

### Gate B：无物单关节往返

1. 仅命令 `ag165_joint1_left` 小幅往返。
2. 看左主动关节、右主动关节和四个被动关节是否连续更新。
3. 看左右指尖是否同向形成“开/合”而不是其中一侧反向。
4. 若异常，先检查 pin pose/axis 与 mimic 状态；不要先改 MoveIt 或深框参数。

### Gate C：接触与抓取

1. 以受限速度接近工件，记录接触 topic，但不把单侧 contact 直接当作双侧稳定抓取。
2. close 指令若在目标角前停止，检查速度、接触、实际位置与 action error；这可能正是夹住工件，而不是控制器必然坏了。
3. 只有经状态机确认的 attach/physical gate 通过后，才开始微抬升。

### Gate D：30 mm 微抬升

对 Demo：验证 follower 的物体 world-Z 位移是否与末端位移一致。  
对 Physical：还需验证工件相对 TCP 滑移、双侧接触或力阈值、抬升后稳定时间。两者不可互换。

### Gate E：遇到“状态正常但不动”的最短排查链

```text
mimic_status: active/follower 是否 resolved？
  → controller 是否 active？
  → joint1_left 的 measured position 是否随 action 变化？
  → mimic 的 target / force 是否随之更新？
  → 右侧 pin 是否有正确 relative_to=link2？
  → 被动五关节是否意外出现 command interface 或 URDF mimic？
  → 再看 CFM、ERP、阻尼或接触摩擦。
```

这比直接增加 effort、增大 `goal_time` 或在 Gazebo UI 对 `link1_left` 反复施力更可靠。后几种做法不能区分“控制器未接管”“过约束死锁”和“销轴几何错误”。

---

## 7. 真机迁移边界

| Gazebo 组件 | 真机对应物 | 迁移时必须重新验证 |
|---|---|---|
| `GazeboGripperSystem` position PID | 大寰夹爪厂商 ROS 2 / Modbus / TCP 驱动 | 开度单位、速度/力参数、到位/故障状态码 |
| SDF pin + ODE | 真实四连杆机构 | 无；真机由机构本身闭合，但要重新标定开度—宽度曲线 |
| mimic plugin | 真实夹爪内部同步机构 | 不应再向左右“虚拟关节”分别发命令 |
| contact bumper | 夹爪电流、力/位置、外部 F/T | 信号含义、滤波、阈值和安全停止 |
| attachment follower | 真实夹持 | 必须删除；改用真实夹持确认和 MoveIt attached object |

真机驱动的寄存器含义、状态码和力/位置单位必须以**对应固件版本的大寰官方通信手册**为准；本文不把未经手册核验的 `0/1/2/3` 等数值写成接口事实。

---

## 8. 源码附录：关键模块与对应代码

本节把所有会影响“夹爪是否运动、是否闭环、是否可演示附着”的关键源代码直接放入文档，并按模块解释。文件索引在最后。

### M1 — AG165 树关节与限位

```xml
<!-- ag165_macro.xacro：左主动关节，右侧同样有独立树关节 -->
<joint name="${prefix}ag165_joint1_left" type="revolute">
  <origin xyz="0.036673 -0.00011644 0.098336" rpy="-1.5708 -0.86932 -0.0031749"/>
  <parent link="${prefix}ag165_base_link"/>
  <child link="${prefix}ag165_link1_left"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.05" upper="0.93" effort="50" velocity="2.1"/>
</joint>
```

**注释**：这给 MoveIt/TF 一个无环树。`joint1_left` 是唯一有 ROS command interface 的关节；右侧 `joint1_right` 的实际驱动交给 M3。

### M2 — Gazebo-only SDF 闭环销轴

```xml
<xacro:if value="${use_sdf_closure}">
  <gazebo>
    <joint name="${prefix}ag165_left_link2_to_link3_sdf" type="revolute">
      <pose relative_to="${prefix}ag165_link2_left">
        ${sdf_left_pose_xyz} ${sdf_left_pose_rpy}
      </pose>
      <parent>${prefix}ag165_link2_left</parent>
      <child>${prefix}ag165_link3_left</child>
      <axis><xyz>${sdf_left_axis}</xyz><use_parent_model_frame>false</use_parent_model_frame>
        <limit><lower>-3.14</lower><upper>3.14</upper><effort>50</effort><velocity>2.1</velocity></limit>
      </axis>
      <physics><ode><cfm>1e-4</cfm><erp>0.2</erp></ode></physics>
    </joint>
  </gazebo>
</xacro:if>
```

**注释**：这段是在 Gazebo Classic 将 URDF 转为 SDF 后补上的物理销轴。右侧同构，但当前 `cfm=1e-3`。这些数是当前已验证候选，不是跨时间步、质量、接触刚度都成立的万能值；一次只改一个变量并保留日志。

### M3 — 右主动关节 mimic 的 PD 力矩

```xml
<plugin name="${prefix}ag165_gripper_mimic" filename="libag165_gripper_mimic_plugin.so">
  <enabled>true</enabled>
  <active_joint>${prefix}ag165_joint1_left</active_joint>
  <kp>${2.0 if soft_test else 2.5}</kp>
  <kd>0.35</kd>
  <effort>${2.0 if soft_test else 2.5}</effort>
  <follower>
    <name>${prefix}ag165_joint1_right</name>
    <multiplier>1.0</multiplier>
    <xacro:if value="${soft_test}"><offset>0.0</offset></xacro:if>
  </follower>
</plugin>
```

```cpp
// ag165_gripper_mimic_plugin.cpp：每个 Gazebo world update 执行
const double target = offsets_[i] + multipliers_[i] * active_position;
const double position = followers_[i]->Position(0);
const double velocity = followers_[i]->GetVelocity(0);
const double raw = kp_ * (target - position) - kd_ * velocity;
const double force = std::max(-effort_, std::min(effort_, raw));
followers_[i]->SetForce(0, force);
```

**注释**：目前 `ag165_soft_mimic_test=true` 时，源码是 `kp=2.0`、`kd=0.35`、`effort=2.0`；不是早期笔记中的 `kp=1.5`。插件默认值可为 1.5，但运行参数由上面的 xacro 插件配置覆盖。`SetForce` 说明这里是物理力矩跟随，而非 ROS 控制器第二次下位置命令。

### M4 — ros2_control 的双硬件组件与唯一命令接口

```xml
<ros2_control name="GazeboGripperSystem" type="system">
  <hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>
  <joint name="ag165_joint1_left">
    <param name="pos_kp">2.5</param><param name="pos_ki">0.0</param><param name="pos_kd">0.35</param>
    <command_interface name="position"/>
    <state_interface name="position"/><state_interface name="velocity"/>
  </joint>
  <joint name="ag165_joint2_left"><state_interface name="position"/><state_interface name="velocity"/></joint>
  <joint name="ag165_joint3_left"><state_interface name="position"/><state_interface name="velocity"/></joint>
  <joint name="ag165_joint1_right"><state_interface name="position"/><state_interface name="velocity"/></joint>
  <joint name="ag165_joint2_right"><state_interface name="position"/><state_interface name="velocity"/></joint>
  <joint name="ag165_joint3_right"><state_interface name="position"/><state_interface name="velocity"/></joint>
</ros2_control>
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <hold_joints>true</hold_joints>
    <parameters>$(find cr7as_grasp)/config/ag165_gazebo_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**注释**：`GazeboArmSystem` 与 `GazeboGripperSystem` 分开，是为了避免机械臂 direct position 与夹爪 PID 控制模式互相干扰。其余五个夹爪关节只有 state interface，正是让 ODE 有自由度解闭环。

### M5 — 固定 TCP：规划、附着和视觉链的公共参考

```xml
<joint name="grasp_tcp_j" type="fixed">
  <origin xyz="0 0.00071813 0.18132" rpy="0 0 0"/>
  <parent link="gripper_base"/>
  <child link="grasp_tcp"/>
</joint>
```

**注释**：TCP 对 `gripper_base` 固定，随整个机械臂运动。不要把它挂到 `link2_left/right` 等活动手指上，否则开爪会改变抓取目标、附着变换和 MoveIt target 的含义。

### M6 — Demo 附着：先冻结物理竞争，再由 follower 保持姿态

```cpp
// demo_attachment_physics_plugin.cpp：所有 Gazebo link 操作都在 world update 线程执行
if (enable_kinematic_demo_hold) {
  link_->SetGravityMode(false);
  for (const auto & collision : link_->GetCollisions()) {
    if (collision) collision->SetCollideBits(0u);
  }
} else {
  link_->SetGravityMode(true);
  for (const auto & collision : link_->GetCollisions()) {
    if (collision) collision->SetCollideBits(0xFFFFu);
  }
}
```

```python
# attachment_follower.py：attach 时建立唯一的相对变换快照
relative_pose = do_transform_pose_stamped(object_in_world, transform_parent_from_world)
relative_pose.header.frame_id = self.parent_frame
enabled, detail = self._set_demo_attachment_physics(True)
if not enabled:
    ...  # 拒绝 attach
self.relative_pose = relative_pose
self.attached = True
self._publish_attachment_snapshot(relative_pose)
```

```python
# attachment_follower.py：attached 时按 T_world_tcp × T_tcp_object 写 Gazebo 实体状态
object_in_world = do_transform_pose_stamped(relative_pose, transform_world_from_parent)
state = EntityState()
state.name = self.object_model
state.pose = object_in_world.pose
state.twist = Twist()
self.pending_set_state = self.set_state_client.call_async(request)
```

**注释**：physics plugin 只负责关/开重力与碰撞，**不负责启动或停止 follower**；follower 节点才持有 `attached` 状态并持续写 pose。Detach 时 follower 先取消随动、清零速度，再恢复普通物理。MoveIt AttachedCollisionObject 应订阅/读取同一个 snapshot，而非假设零相对偏置。

### M7 — 启动时为何要求带 state world

```python
# gazebo_ag165_control.launch.py
# 默认 gazebo world 没有 state 插件；follower 依赖两个服务。
state_world = package_share / "worlds" / "empty_with_state.world"
gazebo_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gzserver.launch.py")),
    launch_arguments={"world": str(state_world)}.items(),
)
```

```xml
<!-- empty_with_state.world -->
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros><namespace>/</namespace></ros>
</plugin>
```

**注释**：缺少该 world plugin 时，`/get_entity_state`、`/set_entity_state` 不可用，Demo follower 无法建立物件快照或写入随动位姿。这是“服务不存在”的根因，不是 TF 或 MoveIt 本身的错误。

---

## 9. 源文件索引（按模块阅读）

| 模块 | 源文件 | 建议优先阅读的内容 | 依赖/影响 |
|---|---|---|---|
| M1/M2/M3 | `src/cr7as_grasp/urdf/ag165_macro.xacro` | 六个树关节、SDF pin、接触传感器、mimic plugin 参数 | 机构拓扑、Gazebo 闭环、碰撞体 |
| M4/M5 | `src/cr7as_grasp/urdf/cr7as_ag165_gazebo.urdf.xacro` | `grasp_tcp_j`、双 ros2_control、`hold_joints` | 控制模式、TF、规划 TCP |
| M4 | `src/cr7as_grasp/config/ag165_gazebo_controllers.yaml` | `gripper_controller` 的 action 容差 | action 成功/超时判定 |
| M3 | `src/cr7as_grasp/src/ag165_gripper_mimic_plugin.cpp` | `Load()`、`OnUpdate()`、`mimic_status` | 右侧跟随、PD 力矩、诊断服务 |
| M6 | `src/cr7as_grasp/src/demo_attachment_physics_plugin.cpp` | world-update 线程中的 gravity/collision 切换 | Demo 附着物理隔离 |
| M6 | `src/cr7as_gazebo_grasp/cr7as_gazebo_grasp/attachment_follower.py` | attach snapshot、follow callback、detach | Demo 位姿随动、MoveIt 几何同源 |
| M7 | `src/cr7as_grasp/launch/gazebo_ag165_control.launch.py` | xacro mappings、state world、controller spawn | 启动参数、Gazebo 插件路径 |
| M7 | `src/cr7as_grasp/worlds/empty_with_state.world` | `libgazebo_ros_state.so` | 实体状态读写服务 |
| 参考模型 | `src/cr7as_grasp/models/ag165_90_4/model.sdf` | 独立夹爪的 SDF 对照 | 单机检查；不要与整机重复 spawn |
| 构建 | `src/cr7as_grasp/CMakeLists.txt` | 两个 Gazebo ModelPlugin target/install | `libag165_gripper_mimic_plugin.so` 等生成路径 |

### 最终排障优先级

```text
服务/插件是否加载
  → controller 是否 active
  → joint1_left 是否实际变化
  → mimic_status 是否有 follower 和目标力矩
  → SDF pin pose/axis 是否在正确局部系
  → 六关节状态是否写入 MoveIt 碰撞场景
  → attach snapshot 与 AttachedCollisionObject 是否一致
  → 再讨论深框轨迹或 PID 微调
```

这条顺序能避免把“夹爪本体模型错误、Demo 附着机制、MoveIt 规划失败、深框几何不可达”混成同一个问题。
