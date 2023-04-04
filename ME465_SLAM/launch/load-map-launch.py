from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_share = get_package_share_directory("ME465_SLAM")
    return LaunchDescription([
        DeclareLaunchArgument(
            name="map_file",
            default_value=os.path.join(slam_share, "map", "sim.yaml"),
            description="Map file to use",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                {"yaml_filename": LaunchConfiguration("map_file")},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            parameters=[{
                # "autostart": True,
                "node_names": ["/map_server"],
            }],
        ),
    ])
