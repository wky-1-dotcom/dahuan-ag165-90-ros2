from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "cr7as_grasp",
            package_name="cr7as_grasp_moveit_config",
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    generated = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        SetParameter(
            name="use_sim_time",
            value=True,
        ),
        *generated.entities,
    ])
