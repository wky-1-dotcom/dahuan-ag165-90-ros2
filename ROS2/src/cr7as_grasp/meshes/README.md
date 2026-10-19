# 网格资源

- 顶层 `b0.STL`、`l1.STL` 至 `l6.STL`：CR7AS 机械臂网格。
- `gripper_base.STL`、`grasp_tcp.STL`：旧/通用末端网格资源。
- `ag165/`：当前 AG165-90 Xacro 使用的夹爪网格。
- `ag95/`：保留的 AG95 历史网格，不是 AG165-90 运行模型。

网格按 Git LFS 存储；碰撞和视觉几何的引用关系应从 Xacro/SDF 中核对，不能仅按文件名替换。
