import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    pkg_capra_action_mapper = get_package_share_directory("capra_actions_mapper")

    actions_config_path = os.path.join(
        pkg_capra_action_mapper, "config", "config.yaml"
    )

    mapper_node = Node(
        package="capra_actions_mapper",
        executable="mapper",
        name="actions_mapper",
        output="screen",
        parameters=[actions_config_path],
        remappings=[
            ("flippers", "flippers_cmd"),
            ("tracks", "tracks_cmd"),
        ]
    )

    return LaunchDescription(
        [
            mapper_node,
        ]
    )