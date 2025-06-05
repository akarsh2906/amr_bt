#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoToPose : public BT::StatefulActionNode
{
public:
  GoToPose(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};


class PickItem : public BT::SyncActionNode
{
public:
  PickItem(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("PickItem"), "Picking up item...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(rclcpp::get_logger("PickItem"), "Pick complete");
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {};
  }
};

class DropItem : public BT::SyncActionNode
{
public:
  DropItem(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("DropItem"), "Dropping item...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(rclcpp::get_logger("DropItem"), "Drop complete");
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {};
  }
};





class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_sub_;
  int battery_level_;
  void battery_callback(const std_msgs::msg::Int32::SharedPtr msg);
};


class SimulateCharging : public BT::StatefulActionNode
{
public:
  SimulateCharging(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr charge_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_sub_;
  int battery_level_;
  bool charging_started_;

  void batteryCallback(const std_msgs::msg::Int32::SharedPtr msg);
};


