#ifndef DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#define DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <functional> 

#include "rclcpp/rclcpp.hpp"

namespace ddsm115_ros2_driver {
enum class ControlLoopModes : uint8_t {
  MODE_NONE = 0x00,
  MODE_CURRENT = 0x01,
  MODE_VELOCITY = 0x02,
  MODE_POSITION = 0x03
};
class DDSM115DriverClient
{
public:
  // Constructor
  DDSM115DriverClient(rclcpp::Logger logger, std::function<void(const std::vector<uint8_t>&)> feedback_callback = nullptr);
  // Destructor
  ~DDSM115DriverClient();

  bool init_port(const std::string &port_name);
  bool reinitialize_port();
  void close_port();

  void send_mode_command(uint8_t motor_id, ControlLoopModes mode);

  // Motor control functions
  bool send_current_command(uint8_t motor_id, double current);
  bool send_velocity_command(uint8_t motor_id, double rpm, bool brake = false);
  bool send_position_command(uint8_t motor_id, double position);

  // Helper functions
  void start_async_read();
  void parse_buffer();
  void process_feedback_packet(const std::vector<uint8_t> &packet);
  bool wait_for_feedback_response(uint8_t motor_id, int timeout_ms = 20);

private:
  std::string port_name_;
  int baud_rate_;
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_;

  uint8_t calc_crc8_maxim(const std::vector<uint8_t> &data);
  bool send_rotate_command(std::vector<uint8_t> &command, uint8_t motor_id);

  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 64> read_buf_;
  std::atomic<bool> reading_;
  std::thread io_thread_;
  std::mutex send_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wait_cv_;
  std::promise<uint8_t> motor_id_promise_;

  std::chrono::steady_clock::time_point feedback_received_time_;
  std::atomic<uint8_t> last_motor_id_{0};

  std::function<void(const std::vector<uint8_t>&)> feedback_callback_;
};
} // namespace ddsm115_ros2_driver
#endif // DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_XP