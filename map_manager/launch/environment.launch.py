from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package's share directory
    package_share_directory = get_package_share_directory("map_manager")

    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(package_share_directory, 'config', 'config.rviz')
    random_map_config_path = os.path.join(package_share_directory, 'config', 'random_map.yaml')
    grid_map_config_path = os.path.join(package_share_directory, 'config', 'grid_map.yaml')

    return LaunchDescription(
        [
            Node(
                package="random_map",
                executable="random_forest",
                name="random_forest",
                parameters=[random_map_config_path],
            ),
            Node(
                package="grid_map",
                executable="grid_map",
                name="grid_map",
                parameters=[grid_map_config_path],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    rviz_config_path,
                ],  # Optional: specify your RViz config file
            ),
        ]
    )
