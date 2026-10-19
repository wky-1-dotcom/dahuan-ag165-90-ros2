# Gazebo ModelPlugin 源码

- `ag165_gripper_mimic_plugin.cpp`：令右主动关节以 PD 力矩跟随左主动关节，并提供 `mimic_status` 诊断服务。
- `demo_attachment_physics_plugin.cpp`：仅为演示附着物体切换重力/碰撞；它不证明真实摩擦夹持，也不控制 AG165 follower。

这两个动态库由包内 `CMakeLists.txt` 构建。修改插件后必须重新构建、重新 source 工作空间并完全重启 Gazebo。
