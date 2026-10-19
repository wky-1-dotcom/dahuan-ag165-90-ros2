from pathlib import Path
from xml.dom import Node as DomNode

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _remove_xml_comments(node):
    """Remove source comments and xacro's generated comment from the DOM."""
    for child in list(node.childNodes):
        if child.nodeType == DomNode.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.hasChildNodes():
            _remove_xml_comments(child)


def _load_robot_description(xacro_path: Path) -> str:
    document = xacro.process_file(str(xacro_path))
    _remove_xml_comments(document)
    xml_text = document.toxml()
    if xml_text.startswith("<?xml") and "?>" in xml_text:
        xml_text = xml_text.split("?>", 1)[1]
    return xml_text.strip()


def generate_launch_description():
    package_share = Path(get_package_share_directory("cr7as_grasp"))
    xacro_path = package_share / "urdf" / "cr7as_grasp.urdf.xacro"
    rviz_path = package_share / "rviz" / "display.rviz"
    robot_description = _load_robot_description(xacro_path)

    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_path)],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )

