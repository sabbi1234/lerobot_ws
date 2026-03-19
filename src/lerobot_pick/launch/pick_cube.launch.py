import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Robot description (URDF)
    robot_description_content = os.popen(
        "xacro "
        + os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf/so101.urdf.xacro",
        )
    ).read()
    robot_description = {"robot_description": robot_description_content}

    # SRDF
    srdf_path = os.path.join(
        get_package_share_directory("lerobot_moveit"), "config/so101.srdf"
    )
    with open(srdf_path) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Kinematics
    kinematics = load_yaml("lerobot_moveit", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics}

    # Joint limits
    joint_limits = load_yaml("lerobot_moveit", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits}

    # MoveItCpp config
    moveit_cpp_yaml = load_yaml("lerobot_pick", "config/moveit_cpp.yaml")

    # Controllers
    moveit_controllers = load_yaml("lerobot_moveit", "config/moveit_controllers.yaml")

    pick_node = Node(
        package="lerobot_pick",
        executable="pick_cube",
        name="pick_cube",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_cpp_yaml,
            moveit_controllers,
            {"use_sim_time": True},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("lerobot_moveit"), "config/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_node, rviz_node])
