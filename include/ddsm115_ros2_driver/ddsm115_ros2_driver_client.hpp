#ifndef DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#define DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#include <boost/asio.hpp>

class DDSM115DriverClient
{
public:
  DDSM115DriverClient();
  DDSM115DriverClient(const std::string & port, int baud_rate);


  void initialize();

private:
    std::string port_;
    int baud_rate_;
    
};
#endif  // DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_XP