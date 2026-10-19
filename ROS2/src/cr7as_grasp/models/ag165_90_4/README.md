# 独立 AG165-90 SDF 参考模型

- `model.config`：Gazebo 模型元数据与入口声明。
- `model.sdf`：七个 link 的独立模型、树关节、两条 SDF 闭环 pin、右主动关节 mimic 插件以及物理参数。
- `meshes/`：SDF 使用的 AG165-90 网格副本。

该模型用于闭环几何与插件的独立检查。整机运行请使用 `urdf/cr7as_ag165_gazebo.urdf.xacro`，不要双重 spawn。
