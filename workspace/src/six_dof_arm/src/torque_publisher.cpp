/*#include <gazebo/transport/transport.hh>*/
/*#include <gazebo/msgs/msgs.hh>*/
/*#include <gazebo/gazebo_client.hh>*/
/**/
/*#include <rclcpp/rclcpp.hpp>*/
/*#include <geometry_msgs/msg/wrench_stamped.hpp>*/
/**/
/*#include <boost/function.hpp>*/
/*#include <boost/bind.hpp>*/
/**/
/*#include <map>*/
/*#include <string>*/
/*#include <vector>*/
/**/
/*// Global map to store ROS publishers for each joint's force-torque data*/
/*std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> publishers;*/
/**/
/*class GazeboTransportToRos : public rclcpp::Node*/
/*{*/
/*public:*/
/*  GazeboTransportToRos() : Node("multi_force_torque_bridge")*/
/*  {*/
/*    // Initialize Gazebo client and node*/
/*    gazebo::client::setup(0, nullptr);*/
/*    gz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());*/
/*    gz_node->Init();*/
/**/
/*    // List of joints to bridge*/
/*    std::vector<std::string> joints = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};*/
/**/
/*    for (const auto& joint : joints)*/
/*    {*/
/*      std::string gz_topic = "~/" + joint + "_force_torque";*/
/*      std::string ros_topic = "/" + joint + "_force_torque";*/
/**/
/*      // Create a ROS 2 publisher for each joint's topic*/
/*      publishers[ros_topic] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(ros_topic, 10);*/
/**/
/*      // Subscribe to the Gazebo topic using a lambda that captures 'this' and 'ros_topic'*/
/*      gz_node->Subscribe(gz_topic, [this, ros_topic](const std::string& msg) {*/
/*        this->stringCallback(msg, ros_topic);*/
/*      });*/
/*    }*/
/*  }*/
/**/
/*  ~GazeboTransportToRos()*/
/*  {*/
/*    gazebo::client::shutdown();*/
/*  }*/
/**/
/*private:*/
/*  // Member function to handle serialized messages*/
/*  void stringCallback(const std::string& serialized_msg, const std::string& ros_topic)*/
/*  {*/
/*    // Parse the serialized message into WrenchStamped*/
/*    gazebo::msgs::WrenchStamped msg;*/
/*    if (!msg.ParseFromString(serialized_msg))*/
/*    {*/
/*      RCLCPP_ERROR(this->get_logger(), "Failed to parse WrenchStamped message from Gazebo");*/
/*      return;*/
/*    }*/
/**/
/*    // Convert to ROS message*/
/*    geometry_msgs::msg::WrenchStamped out;*/
/*    out.header.stamp = this->get_clock()->now();*/
/*    out.wrench.force.x = msg.wrench().force().x();*/
/*    out.wrench.force.y = msg.wrench().force().y();*/
/*    out.wrench.force.z = msg.wrench().force().z();*/
/*    out.wrench.torque.x = msg.wrench().torque().x();*/
/*    out.wrench.torque.y = msg.wrench().torque().y();*/
/*    out.wrench.torque.z = msg.wrench().torque().z();*/
/**/
/*    // Publish to the corresponding ROS topic*/
/*    publishers[ros_topic]->publish(out);*/
/*  }*/
/**/
/*  gazebo::transport::NodePtr gz_node;*/
/*};*/
/**/
/*int main(int argc, char** argv)*/
/*{*/
/*  rclcpp::init(argc, argv);*/
/*  rclcpp::spin(std::make_shared<GazeboTransportToRos>());*/
/*  rclcpp::shutdown();*/
/*  return 0;*/
/*}*/
