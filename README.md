# DH Robotics AG165-90 — ROS 2 / Gazebo SDF Closed Loop

This repository packages the CR7AS-mounted DH Robotics AG165-90 gripper assets used for ROS 2 Humble and Gazebo Classic development. It preserves the source CAD assembly, ROS 2 robot-description and MoveIt configuration packages, and the validated SDF closed-loop reference.

## Contents

```text
CAD/SolidWorks/CR7AS+AG165-90/        Original SolidWorks parts and assemblies
ROS2/src/cr7as_grasp/                 URDF/Xacro, AG165 meshes, SDF model, plugins and controllers
ROS2/src/cr7as_grasp_moveit_config/   MoveIt 2 configuration for the CR7AS + AG165 integration
docs/                                 Closed-loop architecture, validation and migration notes
```

The independent Gazebo model is at `ROS2/src/cr7as_grasp/models/ag165_90_4/model.sdf`. The six-link tree used by ROS/MoveIt is defined in `ROS2/src/cr7as_grasp/urdf/ag165_macro.xacro`; the Gazebo-only SDF pins close the four-bar linkage at simulation time.

## Important model boundary

URDF/MoveIt use a tree: only `ag165_joint1_left` receives ROS 2 position commands. Gazebo closes each finger's four-bar linkage using SDF pin joints, while a PD mimic plugin drives `ag165_joint1_right`. Do not command the five passive AG165 joints or add the SDF closure pins to a MoveIt planning group.

See [the closed-loop guide](docs/AG165_SDF闭环_架构坐标与故障复盘学习指南.md) for coordinate conventions, control ownership, validated parameters and troubleshooting gates.

## Large files

SolidWorks and mesh files are stored with Git LFS. After cloning, install and enable Git LFS before fetching the project files:

```bash
git lfs install
git clone https://github.com/wky-1-dotcom/dahuan-ag165-90-ros2.git
cd dahuan-ag165-90-ros2
git lfs pull
```

## STEP notice

No AG165-90 `.stp` or `.step` source was supplied in the source folders used to create this repository. The `CAD/SolidWorks` directory intentionally preserves the original SolidWorks source instead. Add an exported AG165 STEP file later under `CAD/STEP/` when it is available.

## ROS 2 usage

The current files target ROS 2 Humble with Gazebo Classic. Read `docs/Ubuntu迁移与验证.md` before building or launching, and keep generated `build/`, `install/`, and `log/` directories outside version control.
