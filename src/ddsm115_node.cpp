#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class DDSM115Node : public rclcpp::Node
{
public:
  DDSM115Node()
  : Node("ddsm115_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("motor_id", 1);
    this->declare_parameter<double>("publish_rate", 10.0);

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    motor_id_ = this->get_parameter("motor_id").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // Create publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "motor_status", 10);

    // Create timer for publishing
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&DDSM115Node::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "DDSM115 Node initialized");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Motor ID: %d", motor_id_);
  }

private:
  void timer_callback()
  {
    // Publish joint state
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back("ddsm115_joint");
    // TODO: Replace with actual motor feedback data
    joint_state_msg.position.push_back(0.0);  // TODO: Read position from motor encoder
    joint_state_msg.velocity.push_back(0.0);  // TODO: Read velocity from motor feedback
    joint_state_msg.effort.push_back(0.0);    // TODO: Calculate from motor current sensors
    
    joint_state_pub_->publish(joint_state_msg);

    // Publish status
    auto status_msg = std_msgs::msg::String();
    // TODO: Implement actual status checking (connected/error/running/idle)
    status_msg.data = "DDSM115 motor running";
    status_pub_->publish(status_msg);
  }

  std::string serial_port_;
  int baud_rate_;
  int motor_id_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDSM115Node>());
  rclcpp::shutdown();
  return 0;
}
