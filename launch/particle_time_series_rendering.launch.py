from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="particle_time_series_rendering",
                executable="particle_time_series_rendering_node",
                name="particle_time_series_rendering",
                output="screen",
            )
        ]
    )
