#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_cube");

// Joint configurations (radians)
// Cube at x=0.20, y=-0.02, z=0.015 in Gazebo world
// Joint 1=-1.37 rotates base slightly right to center gripper on cube (y=-0.02 offset)
// Joint 6 limits: lower=-0.1745, upper=1.7453
const std::vector<double> HOME          = {-1.52,  0.0,  0.0,  0.0,  0.0};
const std::vector<double> PRE_GRASP     = {-1.52, -0.3,  0.5,  0.3,  1.658};  // approach from above
const std::vector<double> GRASP         = {-1.52, -0.07, 0.785, 0.733, 1.658};  // from RViz goal state
const std::vector<double> GRIPPER_OPEN  = {1.2};
const std::vector<double> GRIPPER_CLOSE = {0.05};

bool planAndExecute(
  moveit_cpp::PlanningComponent & planner,
  const moveit_cpp::MoveItCppPtr & moveit_cpp,
  const std::string & group_name,
  const std::vector<double> & joint_positions,
  const std::string & step_name)
{
  planner.setStartStateToCurrentState();

  // Build goal state from current state, override group joints
  auto goal_state = moveit_cpp->getCurrentState();
  goal_state->setJointGroupPositions(group_name, joint_positions);
  goal_state->update();

  // setGoal(RobotState) — Jazzy API takes only the state, no group_name
  planner.setGoal(*goal_state);

  RCLCPP_INFO(LOGGER, "%s: Planning...", step_name.c_str());
  auto plan_result = planner.plan();

  if (!plan_result)
  {
    RCLCPP_ERROR(LOGGER, "%s: Planning FAILED", step_name.c_str());
    return false;
  }

  // execute() takes RobotTrajectoryPtr, blocking=true, controllers={}
  RCLCPP_INFO(LOGGER, "%s: Executing...", step_name.c_str());
  std::vector<std::string> controllers;
  auto status = moveit_cpp->execute(plan_result.trajectory, controllers);
  if (status != moveit_controller_manager::ExecutionStatus::SUCCEEDED)
  {
    RCLCPP_ERROR(LOGGER, "%s: Execution FAILED (status=%s)",
      step_name.c_str(), status.asString().c_str());
    return false;
  }

  RCLCPP_INFO(LOGGER, "%s: Done", step_name.c_str());
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_cube", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Initialise MoveItCpp
  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);

  rclcpp::sleep_for(std::chrono::seconds(2));

  // Add cube as collision object in the planning scene (visible in RViz)
  // Transient local = latched, so RViz receives it even if it connects late
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene",
    rclcpp::QoS(1).transient_local());
  {

    moveit_msgs::msg::CollisionObject cube;
    cube.id = "pick_cube";
    cube.header.frame_id = "world";
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.03, 0.03, 0.03};  // 3cm cube

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.20;
    pose.position.y = -0.02;
    pose.position.z = 0.015;
    pose.orientation.w = 1.0;

    cube.primitives.push_back(box);
    cube.primitive_poses.push_back(pose);

    moveit_msgs::msg::PlanningScene scene_msg;
    scene_msg.world.collision_objects.push_back(cube);
    scene_msg.is_diff = true;

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    scene_pub->publish(scene_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(LOGGER, "Cube added to planning scene");
  }  // cube object stays, publisher lives for node lifetime

  auto arm     = std::make_shared<moveit_cpp::PlanningComponent>("arm",     moveit_cpp);
  auto gripper = std::make_shared<moveit_cpp::PlanningComponent>("gripper", moveit_cpp);

  RCLCPP_INFO(LOGGER, "=== Starting pick sequence ===");

  RCLCPP_INFO(LOGGER, "Step 1: Moving to home");
  planAndExecute(*arm, moveit_cpp, "arm", HOME, "Home");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Step 2: Opening gripper");
  planAndExecute(*gripper, moveit_cpp, "gripper", GRIPPER_OPEN, "Gripper open");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Step 3: Moving to pre-grasp");
  planAndExecute(*arm, moveit_cpp, "arm", PRE_GRASP, "Pre-grasp");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Step 4: Moving to grasp");
  planAndExecute(*arm, moveit_cpp, "arm", GRASP, "Grasp");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Step 5: Closing gripper");
  planAndExecute(*gripper, moveit_cpp, "gripper", GRIPPER_CLOSE, "Gripper close");
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Attach cube (with geometry) to gripper so MoveIt carries it during lift
  {
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.03, 0.03, 0.03};

    geometry_msgs::msg::Pose pose;
    pose.position.y = -0.05;   // offset to gripper tip (jaw extends ~5cm in -y)
    pose.orientation.w = 1.0;

    moveit_msgs::msg::AttachedCollisionObject attach;
    attach.link_name = "jaw";
    attach.object.id = "pick_cube";
    attach.object.header.frame_id = "jaw";
    attach.object.primitives.push_back(box);
    attach.object.primitive_poses.push_back(pose);
    attach.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attach.touch_links = {"jaw", "gripper"};

    moveit_msgs::msg::PlanningScene attach_msg;
    attach_msg.is_diff = true;
    attach_msg.robot_state.attached_collision_objects.push_back(attach);
    scene_pub->publish(attach_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(LOGGER, "Cube attached to gripper");
  }

  RCLCPP_INFO(LOGGER, "Step 6: Lifting cube");
  planAndExecute(*arm, moveit_cpp, "arm", PRE_GRASP, "Lift");

  RCLCPP_INFO(LOGGER, "=== Pick sequence complete ===");

  arm.reset();
  gripper.reset();
  scene_pub.reset();
  moveit_cpp.reset();

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
