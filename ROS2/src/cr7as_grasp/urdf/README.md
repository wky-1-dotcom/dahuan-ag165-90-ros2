# URDF / Xacro 模型入口

- `ag165_macro.xacro`：AG165-90 的六个树关节、网格、Gazebo 闭环 pin、mimic 插件和传感器定义。
- `cr7as_ag165_gazebo.urdf.xacro`：当前 CR7AS + AG165 整机 Gazebo 入口，包含 `grasp_tcp` 和分离的 arm/gripper `ros2_control` 组件。
- `cr7as_grasp*.urdf*`：旧/通用机器人描述版本。
- `ag95_macro*.xacro`：AG95 历史与对照宏，不是 AG165-90 生效模型。

URDF 必须保持树结构；`link2 ↔ link3` 的物理闭合只能由 Gazebo SDF 扩展表达，不能加入 MoveIt 规划组或普通 URDF 父子关系。
