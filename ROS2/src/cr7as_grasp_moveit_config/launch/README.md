# MoveIt 2 启动文件

- `move_group.launch.py`：启动 MoveIt `move_group`。
- `moveit_rviz.launch.py`：启动带 MoveIt 插件的 RViz。
- `rsp.launch.py`：robot_state_publisher 入口。
- `spawn_controllers.launch.py`：启动 ros2_control 控制器。
- `demo.launch.py`、`setup_assistant.launch.py`、`static_virtual_joint_tfs.launch.py`、`warehouse_db.launch.py`：MoveIt 示例、配置助手、虚拟关节与数据库辅助入口。

先启动 AG165 Gazebo 场景与控制器，再启动 `move_group` 和 RViz，避免模型/控制器状态不同步。
