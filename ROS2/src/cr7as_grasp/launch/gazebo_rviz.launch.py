from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("cr7as_grasp"))
    rviz_path = package_share / "rviz" / "display.rviz"

    # Gazebo launch already owns robot_state_publisher and joint states. Starting
    # either of them again here would create competing TF/robot_description data.
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_gazebo",
                output="screen",
                arguments=["-d", str(rviz_path)],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
