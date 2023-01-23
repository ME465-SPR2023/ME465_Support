from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    share = get_package_share_directory("ME465_Description")
    description = SetParameter(
        name="robot_description",
        value=xacro.process(
            os.path.join(
                share,
                "urdf",
                "robot.urdf.xacro",
            )
        ),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
    )
    return LaunchDescription([
        description,
        robot_state_publisher,
    ])
