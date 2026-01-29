#include <chrono>
#include <memory>
#include <string>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_command.hpp"
#include "ddsm115_ros2_driver/msg/ddsm115_status.hpp"
#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

using namespace std::chrono_literals;

class DDSM115DriverNode : public rclcpp::Node
{
public:
  DDSM115DriverNode()
  : Node("ddsm115driver_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{1});

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::vector<int64_t> motor_ids = this->get_parameter("motor_ids").as_integer_array();


    // Initialize driver client
    driver_client_ = std::make_unique<ddsm115_ros2_driver::DDSM115DriverClient>(
      this->get_logger(),
      std::bind(&DDSM115DriverNode::motor_feedback_callback, this, std::placeholders::_1));

    if (!driver_client_->init_port(serial_port_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
      rclcpp::shutdown();
      return;
    }

    for(int64_t motor_id : motor_ids) {
      // Create publishers
      auto status_pub = this->create_publisher<ddsm115_ros2_driver::msg::Ddsm115Status>(
        "motor_" + std::to_string(motor_id) + "/status", 10);
      status_pub_vec_[motor_id] = status_pub;

      // Create subscribers
      auto command_sub = this->create_subscription<ddsm115_ros2_driver::msg::Ddsm115Command>(
        "motor_" + std::to_string(motor_id) + "/command", 10, [this, motor_id](const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg) -> void {
        topic_callback(std::move(msg), motor_id);
      });
      command_sub_vec_[motor_id] = command_sub;

      motor_modes_[motor_id] = ddsm115_ros2_driver::ControlLoopModes::MODE_VELOCITY;  // Default mode
    }

    // Create timer for publishing commands
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
    subscription_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&DDSM115DriverNode::subscription_timer_callback, this));
    
    
    // Create timer for send command handling
    command_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&DDSM115DriverNode::timer_send_command_callback, this));


    
    RCLCPP_INFO(this->get_logger(), "DDSM115 Driver Node initialized");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  }

  ~DDSM115DriverNode() override
  {
    // Stop timers first to prevent callbacks from running during destruction
    if (subscription_timer_) {
      subscription_timer_->cancel();
      subscription_timer_.reset();
    }
    if (command_timer_) {
      command_timer_->cancel();
      command_timer_.reset();
    }
    
    // Clear subscriptions to stop receiving new messages
    command_sub_vec_.clear();
    status_pub_vec_.clear();
    
    // Stop all motors with proper synchronization
    {
      std::lock_guard<std::mutex> lock(mode_mutex_);
      for (const auto& [motor_id, mode] : motor_modes_) {
        if (mode != ddsm115_ros2_driver::ControlLoopModes::MODE_POSITION) {
          if (driver_client_) {
            driver_client_->send_current_command(static_cast<uint8_t>(motor_id), 0.0);  // Stop motor
          }
        }
      }
    }
    
    if (driver_client_) {
      driver_client_->close_port();
    }
  }

private:
  void timer_send_command_callback()
  {
    if(!driver_client_) return;
    std::map<int, ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr> target_msgs;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      target_msgs = received_msgs_; 
    }
    for (const auto& [motor_id, msg] : target_msgs) {
      if (!msg) continue;

      ddsm115_ros2_driver::ControlLoopModes motor_mode;

      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        motor_mode = motor_modes_[motor_id];
      }
      
      auto send_mode = static_cast<ddsm115_ros2_driver::ControlLoopModes>(msg->mode);
      if(motor_mode != send_mode) {
        driver_client_->send_mode_command(static_cast<uint8_t>(motor_id), static_cast<ddsm115_ros2_driver::ControlLoopModes>(msg->mode));
      }
      
      bool result = false;
      switch (send_mode) {
        case ddsm115_ros2_driver::ControlLoopModes::MODE_CURRENT:
          result = driver_client_->send_current_command(static_cast<uint8_t>(motor_id), msg->value);
          break;
        case ddsm115_ros2_driver::ControlLoopModes::MODE_VELOCITY:
          result = driver_client_->send_velocity_command(static_cast<uint8_t>(motor_id), msg->value, msg->brake_mode == ddsm115_ros2_driver::msg::Ddsm115Command::BRAKE_LOCK);
          break;
        case ddsm115_ros2_driver::ControlLoopModes::MODE_POSITION:
          result = driver_client_->send_position_command(static_cast<uint8_t>(motor_id), msg->value);
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown control mode for motor %d", motor_id);
          break;
      }

      if(result) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        received_msgs_[motor_id] = nullptr;  // Clear the command after sending
        RCLCPP_DEBUG(this->get_logger(), "Sent command to motor %d", motor_id);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to send command to motor %d", motor_id);
      }
    }
  }

  void subscription_timer_callback()
  {
    std::map<int, ddsm115_ros2_driver::msg::Ddsm115Status::SharedPtr> publish_msgs;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        publish_msgs = status_msgs_;
    }

    for (const auto& [motor_id, status_msg] : publish_msgs) {
      if (status_msg && status_pub_vec_.count(motor_id)) {
        status_pub_vec_[motor_id]->publish(*status_msg);
      }
    }
  } 

  void topic_callback(const ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr msg, int motor_id)
  {
    if(!driver_client_) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    received_msgs_[motor_id] = msg;
  }

  void motor_feedback_callback(const std::vector<uint8_t>& packet) {
    ddsm115_ros2_driver::msg::Ddsm115Status::SharedPtr status_msg = std::make_shared<ddsm115_ros2_driver::msg::Ddsm115Status>();

    // Parse motor ID
    uint8_t motor_id = packet[0];
    {
      std::lock_guard<std::mutex> lock(mode_mutex_);
      motor_modes_[motor_id] = static_cast<ddsm115_ros2_driver::ControlLoopModes>(packet[1]);
    }
    double current = static_cast<double>(static_cast<int16_t>((packet[2] << 8) | packet[3]) * (8.0 / 32767.0));
    double velocity = static_cast<double>(static_cast<int16_t>((packet[4] << 8) | packet[5]) * (330.0 / 32767.0));
    double position = static_cast<double>(static_cast<int16_t>((packet[6] << 8) | packet[7] ) * (180.0 / 32767.0) + 180.0);
    uint8_t error_code = packet[8];
    status_msg->current = current;
    status_msg->velocity = velocity;
    status_msg->position = position;
    status_msg->error_code = error_code;

    std::lock_guard<std::mutex> lock(status_mutex_);
    status_msgs_[motor_id] = status_msg;
  }

  std::string serial_port_;
  
  std::map<int, rclcpp::Publisher<ddsm115_ros2_driver::msg::Ddsm115Status>::SharedPtr> status_pub_vec_;
  std::map<int, rclcpp::Subscription<ddsm115_ros2_driver::msg::Ddsm115Command>::SharedPtr> command_sub_vec_;
  rclcpp::TimerBase::SharedPtr subscription_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::map<int, ddsm115_ros2_driver::msg::Ddsm115Command::SharedPtr> received_msgs_;
  std::map<int, ddsm115_ros2_driver::ControlLoopModes> motor_modes_;

  std::map<int, ddsm115_ros2_driver::msg::Ddsm115Status::SharedPtr> status_msgs_;

  std::unique_ptr<ddsm115_ros2_driver::DDSM115DriverClient> driver_client_;

  std::mutex status_mutex_;
  std::mutex command_mutex_;
  std::mutex mode_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDSM115DriverNode>());
  rclcpp::shutdown();
  return 0;
}
