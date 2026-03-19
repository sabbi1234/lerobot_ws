"""
Pick cube task for SO101 robotic arm using MoveIt 2.

Uses pymoveit2 MoveIt2 (arm) and MoveIt2Gripper (gripper) with
use_move_group_action=True so planning + execution happen in one step
via /move_action (same as RViz "Plan & Execute").

Cube is at x=0.15, y=0, z=0.015 in the Gazebo world (pick_world.sdf).
Joint 6 limits: lower=-0.1745, upper=1.7453
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.moveit2_gripper import MoveIt2Gripper

ARM_JOINT_NAMES = ["1", "2", "3", "4", "5"]
GRIPPER_JOINT_NAMES = ["6"]

# Arm configurations
HOME        = [0.0,  0.0, 0.0, 0.0, 0.0]
PRE_GRASP   = [0.0, -0.5, 0.8, 0.5, 0.0]
GRASP       = [0.0, -0.3, 1.0, 0.6, 0.0]

# Gripper: joint 6 limits lower=-0.1745, upper=1.7453
GRIPPER_OPEN   = [1.2]   # wide open
GRIPPER_CLOSED = [0.05]  # closed


def main():
    rclpy.init()
    node = Node("pick_cube")
    callback_group = ReentrantCallbackGroup()

    # Arm — use move_group action (plan + execute in one step)
    moveit2 = MoveIt2(
        node=node,
        joint_names=ARM_JOINT_NAMES,
        base_link_name="base",
        end_effector_name="gripper",
        group_name="arm",
        callback_group=callback_group,
        use_move_group_action=True,
    )
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Gripper — MoveIt2Gripper with move_group action
    gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=GRIPPER_JOINT_NAMES,
        open_gripper_joint_positions=GRIPPER_OPEN,
        closed_gripper_joint_positions=GRIPPER_CLOSED,
        gripper_group_name="gripper",
        callback_group=callback_group,
        use_move_group_action=True,
    )
    gripper.max_velocity = 1.0
    gripper.max_acceleration = 1.0

    # Spin executor in a background thread (required by pymoveit2)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Wait for MoveIt to be ready
    node.create_rate(2.0).sleep()

    node.get_logger().info("=== Starting pick sequence ===")

    node.get_logger().info("Step 1: Moving to home")
    moveit2.move_to_configuration(HOME)
    moveit2.wait_until_executed()

    node.get_logger().info("Step 2: Opening gripper")
    gripper.open()
    gripper.wait_until_executed()

    node.get_logger().info("Step 3: Moving to pre-grasp")
    moveit2.move_to_configuration(PRE_GRASP)
    moveit2.wait_until_executed()

    node.get_logger().info("Step 4: Moving to grasp")
    moveit2.move_to_configuration(GRASP)
    moveit2.wait_until_executed()

    node.get_logger().info("Step 5: Closing gripper")
    gripper.close()
    gripper.wait_until_executed()

    node.get_logger().info("Step 6: Lifting cube")
    moveit2.move_to_configuration(PRE_GRASP)
    moveit2.wait_until_executed()

    node.get_logger().info("=== Pick sequence complete ===")

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
