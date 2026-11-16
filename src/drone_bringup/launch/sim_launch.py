from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("drone_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="false",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup, "launch", "vehicle_launch.py"])
                ),
                launch_arguments={
                    "fcu_url": "udp://:14555@",
                    "gcs_url": "",
                    "config": PathJoinSubstitution([bringup, "config", "sim.yaml"]),
                    "record": LaunchConfiguration("record"),
                    "bag_prefix": "sim",
                }.items(),
            ),
        ]
    )
