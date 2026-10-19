# Changelog

## 2026-09-17 — dev_ws_new 文档与接口契约整理

- 从原 `dev_ws` 复制 CR7AS 相关包到独立的 `dev_ws_new`，原工作空间未修改；
- 为 `cr7as_grasp` 新增阶段状态、模型契约、坐标系/TCP、Gazebo、V4 兼容性、验收清单文档；
- 增加硬件、关节映射、工具坐标和安全参数示例配置；
- 明确 CR7AS 相比 CR7 增加皮肤检测，但本 Gazebo 仿真不处理皮肤检测；
- 明确官方 V4 CR7 URDF 的质量、惯性、惯量不作为本 CR7AS Gazebo 仿真依据；保留本包 CR7AS 模型参数。
