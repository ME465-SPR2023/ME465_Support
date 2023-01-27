from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import SetRemap, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_share = get_package_share_directory("ME465_Sim")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "simulation-launch.py"),
            
        )
    )
    slam_share = get_package_share_directory("ME465_SLAM")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(slam_share, "rviz", "slam.rviz"),
        ],
    )
    description_share = get_package_share_directory("ME465_Description")
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "description-launch.py")
        )
    )
    nav_share = get_package_share_directory("nav2_bringup")
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "bringup_launch.py")
        ),
        launch_arguments=[
            ('slam', 'True'),
            ('use_sim_time', 'True'),
            ('map', 'nothing_file')
        ],
    )
    
    return LaunchDescription([
        simulation,
        rviz,
        description,
        GroupAction(
            actions=[
                SetRemap(src="/cmd_vel", dst="/ctrl_vel"),
                slam_bringup,
            ],
        ),
    ])
