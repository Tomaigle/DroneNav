from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    config = LaunchConfiguration("config")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ),
        launch_arguments={
            "fcu_url": fcu_url,
            "gcs_url": gcs_url,
        }.items(),
    )

    vehicle = Node(
        package="drone_vehicle",
        executable="vehicle_node",
        name="vehicle",
        parameters=[config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
            ),
            DeclareLaunchArgument("gcs_url", default_value=""),
            DeclareLaunchArgument("config"),
            mavros,
            vehicle,
        ]
    )
