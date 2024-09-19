import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time = SetParameter(name="use_sim_time", value=False)

    package_dir = FindPackageShare(package="fodom_ros").find("fodom_ros")
    rviz_config_file = os.path.join(package_dir, "config", "viz_logger.rviz")
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_file,
        description="Path to the RViz configuration file",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    fodom_ros_node = Node(
        package="fodom_ros",
        executable="fodom_ros_node",
        name="fodom_ros_node",
        output="screen",
        parameters=[{"frequency": 10.0}],
    )

    synthetic_imu_publisher_node = Node(
        package="fodom_ros",
        executable="synthetic_imu_publisher_node",
        name="synthetic_imu_publisher_node",
        output="screen",
        parameters=[{"frequency": 10.0}],
    )

    # Add actions to the LaunchDescription
    ld.add_action(use_sim_time)
    ld.add_action(rviz_config)
    ld.add_action(fodom_ros_node)
    ld.add_action(synthetic_imu_publisher_node)
    ld.add_action(rviz_node)

    return ld
