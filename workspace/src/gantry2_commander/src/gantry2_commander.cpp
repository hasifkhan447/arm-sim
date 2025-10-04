#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

using moveit::planning_interface::MoveGroupInterface;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

bool plan_and_execute(MoveGroupInterface &move_group, const std::vector<double> &targets, 
                      rclcpp::Logger logger, const std::string& description = "")
{
    RCLCPP_INFO(logger, "Planning to: %s", description.c_str());
    
    move_group.setJointValueTarget(targets);
    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setNumPlanningAttempts(10);
    
    MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));
    
    if (success) {
        RCLCPP_INFO(logger, "Plan succeeded, executing...");
        auto result = move_group.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Execution failed with error code: %d", result.val);
            return false;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed for: %s", description.c_str());
    }
    return success;
}

// Direct gripper control bypassing MoveIt
bool control_gripper(rclcpp::Node::SharedPtr node, double position, rclcpp::Logger logger)
{
    auto gripper_client = rclcpp_action::create_client<FollowJointTrajectory>(
        node, "/ee_controller/follow_joint_trajectory");
    
    if (!gripper_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger, "Gripper action server not available!");
        return false;
    }
    
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"gripper_left_joint", "gripper_right_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.velocities = {0.0, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);  // 2 second movement
    
    goal_msg.trajectory.points.push_back(point);
    
    RCLCPP_INFO(logger, "Sending gripper command: %.3f", position);
    
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    auto goal_handle_future = gripper_client->async_send_goal(goal_msg, send_goal_options);
    
    if (rclcpp::spin_until_future_complete(node, goal_handle_future, std::chrono::seconds(3)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Failed to send gripper goal");
        return false;
    }
    
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(logger, "Gripper goal was rejected");
        return false;
    }
    
    // Wait for result
    auto result_future = gripper_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Failed to get gripper result");
        return false;
    }
    
    RCLCPP_INFO(logger, "Gripper command completed successfully");
    return true;
}

// New: Attach link using service
bool attach_link(rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
{
    auto client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger, "Attach service not available!");
        return false;
    }
    
    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    request->model1_name = "gantry";  // Adjust to your robot model name
    request->link1_name = "left_finger";   // Adjust to your gripper finger link
    request->model2_name = "cardboard_box";
    request->link2_name = "bottom";         // Box base link
    
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Failed to call attach service");
        return false;
    }
    
    RCLCPP_INFO(logger, "Attachment successful");
    return true;
}

// New: Detach link using service
bool detach_link(rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
{
    auto client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(logger, "Detach service not available!");
        return false;
    }
    
    auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    request->model1_name = "gantry";  // Adjust to your robot model name
    request->link1_name = "left_finger";   // Adjust to your gripper finger link
    request->model2_name = "cardboard_box";
    request->link2_name = "bottom";         // Box base link
    
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "Failed to call detach service");
        return false;
    }
    
    RCLCPP_INFO(logger, "Detachment successful");
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "gantry2_commander",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("gantry2_commander");

    // Create MoveGroup for arm only
    auto prismatic_chain = MoveGroupInterface(node, "prismatic_chain");

    // Set planning parameters
    prismatic_chain.setMaxVelocityScalingFactor(0.5);
    prismatic_chain.setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(logger, "Waiting for action servers...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // -----------------------------
    // 0. Open gripper using direct control
    // -----------------------------
    RCLCPP_INFO(logger, "=== OPENING GRIPPER ===");
    if (!control_gripper(node, 0.08, logger)) {  // Adjust 0.08 if your open position differs
        RCLCPP_ERROR(logger, "Failed to open gripper!");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // -----------------------------
    // 1. Move to home
    // -----------------------------
    RCLCPP_INFO(logger, "=== Moving to HOME ===");
    std::vector<double> home_joints = {0.0, 0.0, 0.3, 0.0};
    if (!plan_and_execute(prismatic_chain, home_joints, logger, "Home Position")) {
        RCLCPP_ERROR(logger, "Failed to reach home! Aborting.");
        rclcpp::shutdown();
        return 1;
    }

    // -----------------------------
    // 2. Move above box
    // -----------------------------
    RCLCPP_INFO(logger, "=== Moving ABOVE BOX ===");
    std::vector<double> above_box = {-0.3, -0.4, 0.3, 0.0};
    if (!plan_and_execute(prismatic_chain, above_box, logger, "Above Box")) {
        RCLCPP_ERROR(logger, "Failed to move above box! Aborting.");
        rclcpp::shutdown();
        return 1;
    }

    // -----------------------------
    // 3. Descend to grasp height
    // -----------------------------
    RCLCPP_INFO(logger, "=== DESCENDING to grasp height ===");
    
    // Step 1: Mid descent
    std::vector<double> mid_descent = {-0.3, -0.4, 0.30, 0.0};
    plan_and_execute(prismatic_chain, mid_descent, logger, "Mid Descent (z=0.30)");
    
    // Step 2: Grasp height - using your working position; adjust if needed
    std::vector<double> at_box = {-0.26, 0.04, -0.25, 0.0};  // From your message; fingers should align for grasp
    plan_and_execute(prismatic_chain, at_box, logger, "Grasp Height");
    

    RCLCPP_INFO(logger, "At grasp position");

    // -----------------------------
    // 4. Close gripper
    // -----------------------------
    RCLCPP_INFO(logger, "=== CLOSING GRIPPER ===");
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    if (!control_gripper(node, 0.065, logger)) {  // Updated to your 0.65; adjust if 0.0 is correct for close
        RCLCPP_ERROR(logger, "Failed to close gripper!");
    }
    rclcpp::sleep_for(std::chrono::seconds(2));

    // New: Attach box to gripper for stable lift
    RCLCPP_INFO(logger, "=== ATTACHING BOX ===");
    if (!attach_link(node, logger)) {
        RCLCPP_ERROR(logger, "Failed to attach box! Continuing without attachment.");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // -----------------------------
    // 5. Lift box
    // -----------------------------
    RCLCPP_INFO(logger, "=== LIFTING ===");
    std::vector<double> lifted = {-0.3, -0.4, 0.4, 0.0};
    if (!plan_and_execute(prismatic_chain, lifted, logger, "Lift Box")) {
        RCLCPP_ERROR(logger, "Lift failed!");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // -----------------------------
    // 6. Return home
    // -----------------------------
    RCLCPP_INFO(logger, "=== RETURNING HOME ===");
    if (!plan_and_execute(prismatic_chain, home_joints, logger, "Return Home")) {
        RCLCPP_ERROR(logger, "Return home failed!");
    }

    // New: Detach box before release
    RCLCPP_INFO(logger, "=== DETACHING BOX ===");
    if (!detach_link(node, logger)) {
        RCLCPP_ERROR(logger, "Failed to detach box!");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // -----------------------------
    // 7. Open gripper to release
    // -----------------------------
    RCLCPP_INFO(logger, "=== RELEASING BOX ===");
    if (!control_gripper(node, 0.08, logger)) {
        RCLCPP_ERROR(logger, "Failed to open gripper!");
    }

    RCLCPP_INFO(logger, "=================================");
    RCLCPP_INFO(logger, "    SEQUENCE COMPLETED");
    RCLCPP_INFO(logger, "=================================");
    
    rclcpp::shutdown();
    return 0;
}
