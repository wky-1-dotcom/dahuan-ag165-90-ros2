# 03：Gazebo 仿真与控制器说明

## 当前仿真控制器

`config/gazebo_controllers.yaml` 中的控制器仅用于 Gazebo Classic：

```text
/controller_manager
  ├── joint_state_broadcaster
  └── arm_controller
        └── /arm_controller/follow_joint_trajectory
```

关节名称为：

```text
j1, j2, j3, j4, j5, j6
```

`cr7as_grasp_moveit_config` 的 MoveIt 控制器同样指向 `arm_controller`，因此 P3.1 可以在仿真中只规划/显示轨迹。

## 与官方 V4 实机控制器的关系

越疆 V4 官方 CR7 参考配置使用：

```text
/cr7_group_controller/follow_joint_trajectory
```

并由 `action_move_server` 将 `FollowJointTrajectory` 转为控制器 `ServoJ` 调用。CR7AS 按已确认的 CR7 对照关系，硬件控制器名称与 CR7 保持一致；但本项目不能把 Gazebo 的 `arm_controller` 直接改成/当成实机控制器。

正确关系为：

```text
仿真：MoveIt -> /arm_controller/follow_joint_trajectory -> Gazebo
实机（未来）：MoveIt -> /cr7_group_controller/follow_joint_trajectory -> V4 action_move_server -> ServoJ
```

## 动力学参数限制

官方 CR7 V4 URDF 的质量、惯性、惯量不适用于本 CR7AS Gazebo 仿真，**不采纳**。本仿真继续使用 `cr7as_grasp_gazebo.urdf` 中既有 CR7AS 参数。

同时应注意：

- 现有 CR7AS inertial 参数是仿真模型参数，非真实硬件认证数据；
- `effort=10.0`、`velocity=1.0`、阻尼和摩擦都是当前仿真初值；
- 不得从 Gazebo 的稳定性直接推导实机负载、最大速度、碰撞容许范围或制动距离。

## 不变原则

- 不修改当前 `j1..j6` 仿真控制器命名；
- 不用官方 CR7 URDF 覆盖 CR7AS 的工具链、质量、惯性、惯量；
- 官方 V4 只作为未来的 ROS 2 实机通讯、控制器 IK 与可达性检查接口参考；
- 皮肤检测不属于当前 Gazebo 仿真，不能将仿真结果解释为皮肤检测安全功能通过。
