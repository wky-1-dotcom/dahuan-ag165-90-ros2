# AG165 SDF 闭环测量与调试记录模板

## 1. 固定约定

- ROS/Gazebo 长度单位：m
- 角度单位：rad
- 统一初始姿态：`ag165_joint1_left = 0.0`
- 先记录左侧，再记录右侧
- 所有测量值必须说明参考坐标系

## 2. SolidWorks 测量表

| 项目 | 左侧 | 右侧 | 参考坐标系 |
|---|---:|---:|---|
| inner link 原点在 AG165 body 中的位置 | | | body |
| finger link 原点在 AG165 body 中的位置 | | | body |
| pin center 在 inner link 坐标系中的位置 | | | inner |
| pin center 在 finger link 坐标系中的位置 | | | finger |
| pin axis 在 inner link 坐标系中的方向 | | | inner |
| pin axis 在 finger link 坐标系中的方向 | | | finger |
| pin center 在 AG165 body 中的最终位置 | | | body |
| pin axis 在 AG165 body 中的最终方向 | | | body |

## 3. 坐标转换

若 `T_B_I` 是 inner link 到 body 的变换，`T_B_F` 是 finger 到 body 的变换：

```text
P_pin_body_from_inner  = T_B_I * P_pin_inner
P_pin_body_from_finger = T_B_F * P_pin_finger
residual = P_pin_body_from_inner - P_pin_body_from_finger
```

初始姿态下要求：

```text
|residual| < 1e-4 m
```

轴方向转换为：

```text
axis_body_from_inner  = R_B_I * axis_inner
axis_body_from_finger = R_B_F * axis_finger
```

两者应平行；若点积接近 `-1`，只翻转其中一个方向。

## 4. Gazebo 启动测试

先只启用左侧：

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py \
  use_sdf_closure:=true \
  sdf_left_enabled:=true \
  sdf_right_enabled:=false \
  sdf_left_pose_xyz:="x y z" \
  sdf_left_pose_rpy:="r p y" \
  sdf_left_axis:="ax ay az"
```

确认静止状态后，依次测试：

```text
0.01 rad -> 0.02 rad -> 0.05 rad -> 0.10 rad
```

## 5. 每次测试记录

- 是否启动成功：
- 初始姿态是否跳变：
- pin 是否分离：
- finger 是否抖动：
- link 是否穿透：
- `/joint_states` 是否连续：
- controller 是否报 tolerance 错误：
- Gazebo 是否出现约束/ODE 警告：
- 左侧是否稳定：

右侧稳定后再设置：

```bash
sdf_right_enabled:=true
```

不要在没有 CAD 测量值时使用 `use_sdf_closure:=true` 的默认零 pose。
