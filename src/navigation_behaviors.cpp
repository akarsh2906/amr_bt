#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  YAML::Node locations = YAML::LoadFile(location_file);

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize(); // todo: why?
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}






CheckBattery::CheckBattery(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config), node_(node), battery_level_(100)
{
  battery_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/battery_state", 10,
      std::bind(&CheckBattery::battery_callback, this, std::placeholders::_1));
}

void CheckBattery::battery_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  battery_level_ = msg->data;
}

BT::NodeStatus CheckBattery::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Battery level: %d", this->name().c_str(), battery_level_);

  if (battery_level_ < 20)
  {
    RCLCPP_WARN(node_->get_logger(), "[%s] Battery low! Returning FAILURE.", this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}





SimulateCharging::SimulateCharging(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr), charging_started_(false)
{
  charge_client_ = node_ptr_->create_client<std_srvs::srv::Trigger>("/start_charging");
  battery_sub_ = node_ptr_->create_subscription<std_msgs::msg::Int32>(
      "/battery_state", 10, std::bind(&SimulateCharging::batteryCallback, this, std::placeholders::_1));
  battery_level_ = 0;
}

BT::PortsList SimulateCharging::providedPorts()
{
  return {};
}

void SimulateCharging::batteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  battery_level_ = msg->data;
}


BT::NodeStatus SimulateCharging::onStart()
{
  if (!charging_started_)
  {
    int retries = 5;
    while (!charge_client_->wait_for_service(std::chrono::seconds(1)) && retries > 0)
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "[SimulateCharging] Waiting for charging service...");
      retries--;
    }

    if (retries == 0)
    {
      RCLCPP_ERROR(node_ptr_->get_logger(), "[SimulateCharging] Charging service not available after retries!");
      return BT::NodeStatus::FAILURE;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    charge_client_->async_send_request(req);
    charging_started_ = true;
    RCLCPP_INFO(node_ptr_->get_logger(), "[SimulateCharging] Sent charging request");
  }

  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus SimulateCharging::onRunning()
{
  if (battery_level_ >= 100)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[SimulateCharging] Battery full, charging complete");
    charging_started_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000, "[SimulateCharging] Charging... battery: %d", battery_level_);
  return BT::NodeStatus::RUNNING;
}

