# MoveIt 2 配置索引

- `cr7as_grasp.srdf`：规划组、末端和碰撞禁用关系。
- `cr7as_grasp.urdf.xacro`、`cr7as_grasp.ros2_control.xacro`：MoveIt 引用的机器人与控制接口包装。
- `joint_limits.yaml`、`kinematics.yaml`、`pilz_cartesian_limits.yaml`：速度、运动学和 Pilz 限制。
- `moveit_controllers.yaml`、`ros2_controllers.yaml`：MoveIt 到 ROS 2 控制器的映射。
- `initial_positions.yaml`、`moveit.rviz`：初始姿态和 RViz 界面。

SDF 闭环 pin 不属于该包的规划关节；规划中只能命令 `ag165_joint1_left`。
