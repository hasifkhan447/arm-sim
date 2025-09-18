#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "gantry2_commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gantry2_commander");

  // Next step goes here
  auto prismatic_chain_move_group_interface = MoveGroupInterface(node, "prismatic_chain");




  // std::vector<double> joints = {map_to_xaxis, xaxis_to_yaxis, yaxis_to_zaxis, zaxis_to_eemount}; // order must match group joint_names
  std::vector<double> prismatic_chain_joints = {-0.3, 0.05, -0.4, 3}; // the angle is in radians
  prismatic_chain_move_group_interface.setJointValueTarget(prismatic_chain_joints);

  auto const [prismatic_success, prismatic_plan] = [&prismatic_chain_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(prismatic_chain_move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();


  // Execute the plan
  if(prismatic_success) {
    prismatic_chain_move_group_interface.execute(prismatic_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for prismatic!");
  }

  auto ee_move_group_interface = MoveGroupInterface(node, "ee");

  std::vector<double> ee_joints = {0.01, 0.01}; // order must match group joint_names
  ee_move_group_interface.setJointValueTarget(ee_joints);

  auto const [ee_success, ee_plan] = [&ee_move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(ee_move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();


  // Execute the plan
  if(ee_success) {
    prismatic_chain_move_group_interface.execute(ee_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed for EE!");
  }




  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
