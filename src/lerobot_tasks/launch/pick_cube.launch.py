from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pick_node = Node(
        package="lerobot_tasks",
        executable="pick_cube",
        name="pick_cube",
        output="screen",
    )

    return LaunchDescription([pick_node])
