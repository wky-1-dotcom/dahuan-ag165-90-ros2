# Gazebo 世界文件

`empty_with_state.world` 在空世界基础上加载 `libgazebo_ros_state.so`。它为演示附着机制提供实体状态读写服务；未加载时 `/get_entity_state` 与 `/set_entity_state` 不可用。

该服务只支撑 Demo 物体随动，不代表真实夹持通过验证。
