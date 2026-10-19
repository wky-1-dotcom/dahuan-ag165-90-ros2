# CR7AS + DH Robotics AG165 integration

Generated files:

- `urdf/ag165_macro.xacro`: AG165 macro derived from the RViz-validated `AG165.revolute_limits.urdf`; all names are prefixed internally as `ag165_*`.
- `urdf/cr7as_ag165_gazebo.urdf.xacro`: CR7AS arm plus AG165 and `gazebo_ros2_control`.
- `config/ag165_gazebo_controllers.yaml`: six arm joints plus one commanded gripper joint, `ag165_joint1_left`.
- `launch/gazebo_ag165_control.launch.py`: Gazebo Classic launch with `use_sdf_closure` and closure-pose arguments.

## Stable baseline first

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py use_sdf_closure:=false
```

The baseline uses the AG165 URDF mimic chain. Send a position goal to:

```text
/gripper_controller/joint_trajectory
joint: ag165_joint1_left
```

For example, test `0.02`, `0.05`, `0.10`, then `0.20` rad. Keep the target within the exported limits (`0.0` to `0.62` rad for the active left joint).

## SDF closure test

The two physics-only joints connect `link2_left -> link3_left` and `link2_right -> link3_right`. They are disabled by default because their exact pin-center pose and axis must come from the SolidWorks assembly.

Do **not** enable the zero defaults. Once the CAD measurement is converted to the Gazebo model frame:

```bash
ros2 launch cr7as_grasp gazebo_ag165_control.launch.py \
  use_sdf_closure:=true \
  sdf_left_pose_xyz:="x y z" \
  sdf_left_pose_rpy:="r p y" \
  sdf_right_pose_xyz:="x y z" \
  sdf_right_pose_rpy:="r p y"
```

Use SI units: metres and radians. Test the left closure alone first, without editing the macro:\n\n```bash\nros2 launch cr7as_grasp gazebo_ag165_control.launch.py \\n  use_sdf_closure:=true sdf_left_enabled:=true sdf_right_enabled:=false \\n  sdf_left_pose_xyz:="x y z" sdf_left_pose_rpy:="r p y"\n```\n\nOnly after the left loop is stable, use `sdf_right_enabled:=true` and provide the right CAD pose.

## CAD acceptance checks

At the initial AG165 joint position (`ag165_joint1_left = 0`), verify in SolidWorks and in Gazebo:

1. The two endpoints of each SDF joint coincide at the same pin center.
2. The SDF axis is parallel to the real pin axis and points consistently with the URDF joint axis.
3. The finger and inner-link collision meshes do not start with deep overlap.
4. The pin-center residual is less than about `1e-4 m` before increasing contact stiffness.
5. First test with low velocity and no grasp object; only then add a box/cylinder contact object.

The current copied STL meshes are the original exported meshes. For reliable grasping, replace link collisions with simplified watertight collision meshes if the original STL causes excessive contact pairs.


