from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package's share directory
    package_share_directory = get_package_share_directory("map_manager")

    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(package_share_directory, "config", "config.rviz")

    return LaunchDescription(
        [
            Node(
                package="random_map",
                executable="random_forest",
                name="random_forest",
                parameters=[
                    {
                        "init_state_x": -19.0,
                        "init_state_y": 0.0,
                        "map/x_size": 40,
                        "map/y_size": 20,
                        "map/column_num": 50,
                        "map/cricle_num": 30,
                        "obstacle_shape/lower_rad": 0.5,
                        "obstacle_shape/upper_rad": 0.7,
                        "obstacle_shape/lower_height": 0.0,
                        "obstacle_shape/upper_height": 8.0,
                        "obstacle_shape/radius_l": 0.8,
                        "obstacle_shape/radius_h": 0.5,
                        "obstacle_shape/z_l": 2.7,
                        "obstacle_shape/z_h": 2.8,
                        "obstacle_shape/theta": 0.6,
                        "sensing/radius": 50,
                    }
                ],
            ),
            Node(
                package="grid_map",
                executable="grid_map",
                name="grid_map",
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
