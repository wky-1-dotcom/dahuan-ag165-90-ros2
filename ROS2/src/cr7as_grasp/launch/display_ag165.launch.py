from pathlib import Path
from xml.dom import Node as DomNode

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _remove_xml_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == DomNode.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.hasChildNodes():
            _remove_xml_comments(child)


def generate_launch_description():
    share = Path(get_package_share_directory("cr7as_grasp"))
    document = xacro.process_file(
        str(share / "urdf" / "cr7as_ag165_gazebo.urdf.xacro"),
        mappings={"use_sdf_closure": "false"},
    )
    _remove_xml_comments(document)
    description = document.toxml()
    if description.startswith("<?xml") and "?>" in description:
        description = description.split("?>", 1)[1]

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": description, "use_sim_time": False}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description, "use_sim_time": False}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(share / "rviz" / "ag165_gazebo.rviz")],
            parameters=[{"use_sim_time": False}],
        ),
    ])
