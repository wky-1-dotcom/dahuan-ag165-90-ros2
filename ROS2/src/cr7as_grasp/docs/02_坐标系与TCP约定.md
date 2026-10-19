# 02：坐标系与 TCP 约定

## 固定坐标系链

```text
world
  └── base_link
        └── b0
              └── j1 ... j6
                    └── l6
                          └── gripper_base
                                └── grasp_tcp
```

- `base_link`：项目统一参考坐标系；P2 和 P3 的目标位姿必须在此坐标系表达；
- `l6`：项目机械臂第六轴末端 link；
- `gripper_base`：夹爪固定安装基座；
- `grasp_tcp`：抓取的 ROS 工具中心点，也是当前 P3 的 `end_effector_link`。

## 视觉坐标系链

```text
base_link
  └── virtual_camera_link
        └── virtual_camera_optical_frame
```

P1 输出的 `PointStamped` 中 `point.x/point.y` 是像素 `u/v`，并非米；P2 只有在使用相机内参、TF 与 `plane_z_m` 后，才输出：

```text
/vision/object_pose_base   geometry_msgs/msg/PoseStamped
header.frame_id = base_link
position unit = m
```

P3 对不在 `base_link` 下的 P2 输入位姿应拒绝处理。

## 单位与姿态约定

| 数据 | 约定 |
|---|---|
| 长度 | 米（m） |
| ROS 关节位置 | 弧度（rad） |
| 越疆 V4 服务中的关节/姿态参数 | 接入前必须按官方服务定义单独确认，不与 ROS 内部单位混淆 |
| 四元数顺序 | `[x, y, z, w]` |
| 坐标系 | ROS 右手系 |

## TCP / Tool ID：当前状态

`grasp_tcp` 是 ROS 模型坐标系，**不是已经验证的越疆控制器 Tool TCP**。

当前待确认项：

1. 实际第六轴法兰到 `grasp_tcp` 的平移与旋转；
2. 越疆控制器中对应的 Tool ID；
3. 控制器 Tool TCP 是否与 ROS `grasp_tcp` 完全重合；
4. 夹爪、转接板及工件对应的 payload 与重心；
5. P3 的抓取姿态四元数是否对应真实夹爪的接近方向。

所有现场值只能写入本地 `config/tool_frames.yaml`，该文件从 `tool_frames.example.yaml` 创建并由 `.gitignore` 忽略。

## 夹爪 IO：当前状态

夹爪 IO 仍待确认。官方 V4 提供 `ToolDO`、`ToolDOInstant`、`SetToolPower` 与 `SetTool485` 等服务参考，但在确认下列信息前不得调用：

- 夹爪类型与通信方式：数字 IO / Tool 485 / 外部控制器；
- 输出通道编号；
- 开、合、断电时的有效电平；
- Tool ID；
- 超时、急停和故障后的安全状态。
