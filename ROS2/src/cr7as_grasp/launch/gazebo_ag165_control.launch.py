import os
from pathlib import Path
from xml.dom import Node as DomNode

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _remove_xml_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == DomNode.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.hasChildNodes():
            _remove_xml_comments(child)


def _load_robot_description(xacro_path: Path, mappings: dict) -> str:
    document = xacro.process_file(str(xacro_path), mappings=mappings)
    _remove_xml_comments(document)
    xml_text = document.toxml()
    if xml_text.startswith("<?xml") and "?>" in xml_text:
        xml_text = xml_text.split("?>", 1)[1]
    return xml_text.strip()


def _launch_setup(context):
    package_share = Path(get_package_share_directory("cr7as_grasp"))
    xacro_path = package_share / "urdf" / "cr7as_ag165_gazebo.urdf.xacro"
    mappings = {
        "use_sdf_closure": LaunchConfiguration("use_sdf_closure").perform(context),
        "sdf_left_pose_xyz": LaunchConfiguration("sdf_left_pose_xyz").perform(context),
        "sdf_left_pose_rpy": LaunchConfiguration("sdf_left_pose_rpy").perform(context),
        "sdf_right_pose_xyz": LaunchConfiguration("sdf_right_pose_xyz").perform(context),
        "sdf_right_pose_rpy": LaunchConfiguration("sdf_right_pose_rpy").perform(context),
        "sdf_left_axis": LaunchConfiguration("sdf_left_axis").perform(context),
        "sdf_right_axis": LaunchConfiguration("sdf_right_axis").perform(context),
        "sdf_left_enabled": LaunchConfiguration("sdf_left_enabled").perform(context),
        "sdf_right_enabled": LaunchConfiguration("sdf_right_enabled").perform(context),
        "ag165_soft_mimic_test": LaunchConfiguration("ag165_soft_mimic_test").perform(context),
    }
    robot_description = _load_robot_description(xacro_path, mappings)

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "cr7as_ag165", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    arm_controller = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["arm_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    gripper_controller = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["gripper_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    return [
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=3.0, actions=[joint_state_broadcaster]),
                TimerAction(period=5.0, actions=[arm_controller]),
                TimerAction(period=7.0, actions=[gripper_controller]),
            ],
        )),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("cr7as_grasp"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    ros2_control_plugin_dir = Path(get_package_prefix("gazebo_ros2_control")) / "lib"
    if not ros2_control_plugin_dir.is_dir():
        raise RuntimeError(
            "gazebo_ros2_control plugin directory does not exist: "
            f"{ros2_control_plugin_dir}. Install ros-humble-gazebo-ros2-control."
        )

    old_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_paths = [str(package_share.parent)] + ([old_model_path] if old_model_path else [])
    old_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    ag165_plugin_dir = Path(get_package_prefix("cr7as_grasp")) / "lib"
    plugin_paths = [str(ros2_control_plugin_dir), str(ag165_plugin_dir)] + ([old_plugin_path] if old_plugin_path else [])

    # The generic gazebo.launch.py starts Gazebo's default world and therefore
    # omits libgazebo_ros_state.so. The contact test and attachment follower
    # both require its /get_entity_state and /set_entity_state services.
    state_world = package_share / "worlds" / "empty_with_state.world"
    if not state_world.is_file():
        raise RuntimeError(f"Gazebo state world file does not exist: {state_world}")
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gzserver.launch.py")),
        launch_arguments={"world": str(state_world)}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gzclient.launch.py"))
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sdf_closure", default_value="true",
                              description="Enable the CAD-verified Gazebo SDF closure joints."),
        DeclareLaunchArgument("sdf_left_pose_xyz", default_value="0.022 0 0"),
        DeclareLaunchArgument("sdf_left_pose_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("sdf_right_pose_xyz", default_value="0.022 0 0"),
        DeclareLaunchArgument("sdf_right_pose_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("sdf_left_axis", default_value="0 0 1"),
        DeclareLaunchArgument("sdf_right_axis", default_value="0 0 1"),
        DeclareLaunchArgument("sdf_left_enabled", default_value="true"),
        DeclareLaunchArgument("sdf_right_enabled", default_value="true"),
        DeclareLaunchArgument("ag165_soft_mimic_test", default_value="true",
                              description="Low-effort +1 follower; false restores the reference +1 profile."),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(model_paths)),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(plugin_paths)),
        gazebo_server,
        gazebo_client,
        OpaqueFunction(function=_launch_setup),
    ])



