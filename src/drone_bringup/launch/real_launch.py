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
                "fcu_url",
                default_value="/dev/ttyACM0:115200",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="udp://@127.0.0.1:14550",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup, "launch", "vehicle_launch.py"])
                ),
                launch_arguments={
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "gcs_url": LaunchConfiguration("gcs_url"),
                    "config": PathJoinSubstitution([bringup, "config", "real.yaml"]),
                }.items(),
            ),
        ]
    )
