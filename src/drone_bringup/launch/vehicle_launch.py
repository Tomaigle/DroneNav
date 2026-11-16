import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RECORD_TOPICS = [
    "/vehicle/state",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/battery",
    "/mavros/statustext/recv",
    "/rosout",
]

BAG_SPLIT_S = "300"


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    config = LaunchConfiguration("config")
    record = LaunchConfiguration("record")
    bag_dir = LaunchConfiguration("bag_dir")
    bag_prefix = LaunchConfiguration("bag_prefix")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = PathJoinSubstitution([bag_dir, [bag_prefix, "_" + stamp]])

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

    rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_path,
            "--max-bag-duration",
            BAG_SPLIT_S,
            *RECORD_TOPICS,
        ],
        output="screen",
        condition=IfCondition(record),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url"),
            DeclareLaunchArgument("gcs_url", default_value=""),
            DeclareLaunchArgument("config"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument(
                "bag_dir", default_value=os.path.join(os.getcwd(), "bags")
            ),
            DeclareLaunchArgument(
                "bag_prefix",
                default_value="run",
            ),
            mavros,
            vehicle,
            rosbag,
        ]
    )
