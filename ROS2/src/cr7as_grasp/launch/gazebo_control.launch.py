import os
from pathlib import Path
from xml.dom import Node as DomNode

import xacro
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _remove_xml_comments(node):
    """Remove XML comments from a xacro DOM, including xacro's generated header."""
    for child in list(node.childNodes):
        if child.nodeType == DomNode.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.hasChildNodes():
            _remove_xml_comments(child)


def _load_robot_description(xacro_path: Path) -> str:
    """Expand one xacro source and return comment-free XML text."""
    try:
        document = xacro.process_file(str(xacro_path))
        _remove_xml_comments(document)
        xml_text = document.toxml()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to expand robot description: {xacro_path}: {exc}"
        ) from exc

    # A ROS 2 parameter is already a Unicode string. Remove the XML declaration
    # so parsers such as gazebo_ros/spawn_entity.py never see an encoding header.
    if xml_text.startswith("<?xml") and "?>" in xml_text:
        xml_text = xml_text.split("?>", 1)[1]
    return xml_text.strip()


def generate_launch_description():
    package_share = Path(get_package_share_directory("cr7as_grasp"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    # Keep all Gazebo processes pointed at the installed package resources.
    old_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_paths = [str(package_share.parent)]
    if old_model_path:
        model_paths.append(old_model_path)
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(model_paths),
    )

    # gazebo_ros2_control is a Gazebo Classic plugin, not only a ROS 2 node.
    # Explicitly add its lib directory so Gazebo can create /controller_manager.
    ros2_control_plugin_dir = Path(
        get_package_prefix("gazebo_ros2_control")
    ) / "lib"
    if not ros2_control_plugin_dir.is_dir():
        raise RuntimeError(
            "gazebo_ros2_control plugin directory does not exist: "
            f"{ros2_control_plugin_dir}. Install ros-humble-gazebo-ros2-control."
        )
    old_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    plugin_paths = [str(ros2_control_plugin_dir)]
    if old_plugin_path:
        plugin_paths.append(old_plugin_path)
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.pathsep.join(plugin_paths),
    )

    # This is the only robot description used by the Gazebo launch. It includes
    # ros2_control and the Gazebo camera; do not replace it with the RViz-only xacro.
    xacro_path = package_share / "urdf" / "cr7as_grasp_gazebo.urdf.xacro"
    robot_description = _load_robot_description(xacro_path)

    # This Humble gazebo.launch.py only declares gui/server. The world and
    # extra_gazebo_args arguments belong to gzserver.launch.py, so include
    # gzserver/gzclient directly instead of passing unsupported arguments to
    # the wrapper launch file.
    state_world = package_share / "worlds" / "empty_with_state.world"
    if not state_world.is_file():
        raise RuntimeError(f"Gazebo world file does not exist: {state_world}")

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gzserver.launch.py")
        ),
        launch_arguments={
            "world": str(state_world),
        }.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gzclient.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "cr7as_grasp",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=0.5, actions=[joint_state_broadcaster]),
                TimerAction(period=1.0, actions=[arm_controller]),
                TimerAction(period=1.5, actions=[gripper_controller]),
            ],
        )
    )

    return LaunchDescription(
        [
            set_gazebo_model_path,
            set_gazebo_plugin_path,
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_robot,
            start_controllers_after_spawn,
        ]
    )



