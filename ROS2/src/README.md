# ROS 2 包索引

- [`cr7as_grasp/`](cr7as_grasp/)：核心模型包。包含 AG165-90 Xacro、独立 SDF、Gazebo 插件、控制器和网格。
- [`cr7as_grasp_moveit_config/`](cr7as_grasp_moveit_config/)：配套的 MoveIt 2 规划、SRDF、控制器和 RViz 配置。

两包需放在同一工作空间中构建。`cr7as_grasp` 的 Gazebo 入口依赖 MoveIt 包之外的 ROS/Gazebo 系统依赖，首次使用请按迁移文档执行 `rosdep` 和构建验证。
