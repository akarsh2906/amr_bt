#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"  // CHANGED HERE

class BatterySimNode : public rclcpp::Node
{
public:
  BatterySimNode() : Node("battery_sim_node"), battery_level_(100), charging_(false)
  {
    battery_pub_ = this->create_publisher<std_msgs::msg::Int32>("/battery_state", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatterySimNode::timer_callback, this));

    charge_srv_ = this->create_service<std_srvs::srv::Trigger>(  // CHANGED HERE
      "/start_charging", std::bind(&BatterySimNode::start_charging, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    if (charging_)
    {
      if (battery_level_ < 100)
        battery_level_ += 5;
      if (battery_level_ >= 100)
        charging_ = false;
    }
    else
    {
      if (battery_level_ > 0)
        battery_level_ -= 1;
    }

    auto msg = std_msgs::msg::Int32();
    msg.data = battery_level_;
    battery_pub_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Battery: %d%%", battery_level_);
  }

  void start_charging(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    charging_ = true;
    RCLCPP_INFO(this->get_logger(), "Charging started");

    res->success = true;
    res->message = "Charging initiated.";
  }

  int battery_level_;
  bool charging_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr charge_srv_;  // CHANGED HERE
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatterySimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
