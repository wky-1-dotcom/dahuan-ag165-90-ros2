# 核心模型配置

- `ag165_gazebo_controllers.yaml`：AG165 左主动关节的 `gripper_controller` 与状态发布配置。
- `gazebo_controllers.yaml`：整机 Gazebo 控制器配置。
- `frame_contract.yaml`：模型坐标系与 TCP 契约。
- `joint_names.yaml`：关节名称清单。
- `hardware.example.yaml`、`joint_name_map.example.yaml`、`safety_limits.example.yaml`、`tool_frames.example.yaml`：真机/集成时参考模板，不是可直接下发真机的默认配置。

不要把五个 AG165 被动关节增加为 trajectory 控制器命令关节，闭环会被过约束。
