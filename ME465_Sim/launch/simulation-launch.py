from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from tempfile import NamedTemporaryFile


def generate_launch_description():
    sim_time = SetParameter(name="use_sim_time", value=True)
    sim_share = get_package_share_directory("ME465_Sim")
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            os.path.join(sim_share, "urdf", "world.sdf"),
            '-s',
            'libgazebo_ros_factory.so',
            '-s',
            'libgazebo_ros_init.so',
        ]
    )
    building = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-f",
            os.path.join(sim_share, "urdf", "panowicz_hall.sdf"),
            "-entity",
            "panowicz_hall",
        ],
    )
    kobuki_description_share = get_package_share_directory("kobuki_description")
    model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.normpath(os.path.join(kobuki_description_share, "..")),
    )
    description_share = get_package_share_directory("ME465_Description")
    global urdf_file
    # urdf_file = NamedTemporaryFile()
    class URDF_File:
        def __init__(self, filename, *args, **kwargs):
            self.file = open(filename, *args, **kwargs)
            self.name = filename
        def __getattr__(self, name):
            return getattr(self.file, name)
    urdf_file = URDF_File("/tmp/robot_description.urdf", "wb")
    urdf_file.write(
        bytes(
            xacro.process(
                os.path.join(
                    description_share,
                    "urdf",
                    "robot.urdf.xacro",
                )
            ),
            "utf-8",
        )
    )
    robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-f",
            urdf_file.name,
            "-entity",
            "robot",
        ],
    )
    return LaunchDescription([
        sim_time,
        model_path,
        gazebo,
        building,
        robot,
    ])
