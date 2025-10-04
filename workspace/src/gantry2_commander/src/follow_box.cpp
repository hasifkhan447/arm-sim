// ROS
#include <rclcpp/rclcpp.hpp>

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
size_t count_ = 0;

// BEGIN_SUB_TUTORIAL publishCommands
// Here is the timer callback for publishing commands. The C++ interface sends commands through internal ROS topics,
// just like if Servo was launched using ServoNode.
void publishCommands()
{
  // First we will publish 100 joint jogging commands. The :code:`joint_names` field allows you to specify individual
  // joints to move, at the velocity in the corresponding :code:`velocities` field. It is important that the message
  // contains a recent timestamp, or Servo will think the command is stale and will not move the robot.
  auto msg = std::make_unique<control_msgs::msg::JointJog>();
  msg->header.stamp = node_->now();
  msg->joint_names.push_back("map_xaxis");
  msg->joint_names.push_back("xaxis_to_yaxis");
  msg->joint_names.push_back("zaxis_to_eemount");
  msg->velocities.push_back(0.03);
  msg->velocities.push_back(0.04);
  msg->velocities.push_back(0.04);
  joint_cmd_pub_->publish(std::move(msg));

}
// END_SUB_TUTORIAL

// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
  // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
  // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10);

  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  RCLCPP_INFO(LOGGER, "Successfully initialized");
  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  servo->start();

  // Sending Commands
  // ^^^^^^^^^^^^^^^^
  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  // CALL_SUB_TUTORIAL publishCommands

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}
