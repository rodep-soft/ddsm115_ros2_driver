#ifndef DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#define DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <functional> 

class DDSM115DriverClient
{
public:
  // Constructor
  DDSM115DriverClient(rclcpp::Logger logger, std::function<void(const std::vector<uint8_t>&)> feedback_callback = nullptr);
  // Destructor
  ~DDSM115DriverClient();

  bool init_port(const std::string &port_name, int baud_rate);
  bool reinitialize_port();

  // Motor control functions
  void send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake = false);

  // Helper functions
  void clear_serial_buffer();
  void start_async_read();
  void parse_buffer();
  void process_feedback_packet(const std::vector<uint8_t> &packet);
  void wait_for_feedback_response(uint8_t motor_id, int timeout_ms = 20);

private:
  std::string port_name_;
  int baud_rate_;
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_;

  uint8_t calc_crc8_maxim(const std::vector<uint8_t> &data);

  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 64> read_buf_;
  std::atomic<bool> reading_;
  std::thread io_thread_;

  std::chrono::steady_clock::time_point feedback_received_time_;
  std::atomic<uint8_t> last_motor_id_{0};

  std::function<void(const std::vector<uint8_t>&)> feedback_callback_;
};
#endif // DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_XP