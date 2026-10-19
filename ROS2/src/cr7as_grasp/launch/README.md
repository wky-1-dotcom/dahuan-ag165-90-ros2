# 启动入口

- `display_ag165.launch.py`：仅展示 AG165 相关模型的入口。
- `gazebo_ag165_control.launch.py`：当前 CR7AS + AG165 SDF 闭环 Gazebo 控制入口。
- `display.launch.py`、`gazebo_control.launch.py`、`gazebo_rviz.launch.py`：历史或通用显示/控制入口；使用前核对是否加载了 `cr7as_ag165_gazebo.urdf.xacro`。

完整场景只能生成一套夹爪：不要同时 spawn 独立 `models/ag165_90_4/model.sdf` 和 Xacro 集成的整机夹爪。
