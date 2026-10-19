# cr7as_grasp_moveit_config 更新记录（2026-09-12）

## 本次变更

### 1. 修正 TCP 质量
修改文件：

- `../urdf/cr7as_grasp.urdf`
- `../urdf/cr7as_grasp_gazebo.urdf`

变更：

- `grasp_tcp` 的质量由 `23.275 kg` 改为 `0.05 kg`（50 g）；
- TCP 惯性原点改为 `0 0 0`；
- 使用小量、对称的近似惯量；
- 保留 TCP 为无 visual、无 collision 的工具坐标 link。

### 2. 修正夹爪底座质量
修改文件：

- `../urdf/cr7as_grasp.urdf`
- `../urdf/cr7as_grasp_gazebo.urdf`

变更：

- `gripper_base` 的质量由导出值 `0.44629 kg` 改为 `2.0 kg`。
- 现有惯量值已按质量比例作仿真近似，后续应使用夹爪真实 CAD 质量属性替换。

### 3. 新增 MoveIt 2 配置包
新增目录：

- `cr7as_grasp_moveit_config/`

关键配置：

- 规划组名称：`ARM`；
- 规划基座：`b0`；
- 规划末端：`grasp_tcp`；
- 规划关节：`j1`～`j6`；
- 控制器：`arm_controller/follow_joint_trajectory`；
- 不加入夹爪开合控制，因为当前夹爪仍是固定安装。

### 4. 明确启动边界

- `cr7as_grasp/launch/gazebo_control.launch.py`：启动 Gazebo 和实际仿真控制器；
- `cr7as_grasp_moveit_config/launch/move_group.launch.py`：启动 MoveIt 规划节点；
- `cr7as_grasp_moveit_config/launch/moveit_rviz.launch.py`：启动 MoveIt RViz；
- 新配置没有 FakeSystem，不应把旧的 `urdf_ros2_moveit_config/demo.launch.py` 与 Gazebo 控制链路混用。

## 尚未解决、需要后续实测

- `grasp_tcp` 相对夹爪的 xyz/rpy 是否与实际两指中心一致；
- 夹爪真实惯量和质心；
- 夹爪简化 collision mesh；
- 夹爪开合关节和真实控制接口；
- 桌子、物体和 attach/detach 抓取节点；
- 相机和视觉坐标变换。

## 回溯方式

修改前的 URDF 备份保存在：

- `../urdf/cr7as_grasp.urdf.bak_20260912`
- `../urdf/cr7as_grasp_gazebo.urdf.bak_20260912`


## 2026-09-12 启动错误补充修复

错误提示：

`pilz_cartesian_limits.yaml doesn't exist`

原因是 MoveItConfigsBuilder 会自动尝试读取该配置文件，而新包中遗漏了它。现已新增：

- `config/pilz_cartesian_limits.yaml`
- `config/moveit.rviz`

本次没有修改规划组、末端或控制器配置。
