#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

#include <bit>
#include <cmath>
#include <mutex>
#include <thread>
#include <iostream>
#include <sstream>

constexpr uint32_t BAUD_RATE = 115200;

namespace ddsm115_ros2_driver
{

  uint8_t DDSM115DriverClient::calc_crc8_maxim(const std::vector<uint8_t> &data)
  {
    uint8_t crc = 0x00; // 初期値  (一般的なMaxim CRCの標準)

    const uint8_t reflected_polynomial = 0x8C;

    // データバイトを一つずつ処理
    for (size_t i = 0; i < data.size(); i++)
    {                 // DATA[0]~DATA[8]まで、合計9バイト
      crc ^= data[i]; // 現在のバイトとCRCレジスタをXOR

      // 各バイトの8ビットを処理 (LSB First)
      for (uint8_t bit = 0; bit < 8; bit++)
      {
        if (crc & 0x01)
        {                                          // 最下位ビットが1の場合
          crc = (crc >> 1) ^ reflected_polynomial; // 右シフトして多項式とXOR
        }
        else
        {
          crc >>= 1; // 最下位ビットが0の場合、単に右シフト
        }
      }
    }
    return crc;
  }

  DDSM115DriverClient::DDSM115DriverClient(FeedbackCallback feedback_callback, LogCallback log_callback)
      : serial_port_(io_context_), buffer_(), reading_(false), feedback_callback_(feedback_callback), log_callback_(log_callback) {}

  DDSM115DriverClient::~DDSM115DriverClient()
  {
    close_port();
  }

  void DDSM115DriverClient::log(LogLevel level, const std::string &message)
  {
    if (log_callback_)
    {
      log_callback_(level, message);
    }
    else
    {
      // Default logging to stderr if no callback is provided
      std::string level_str;
      switch (level)
      {
      case LogLevel::DEBUG:
        level_str = "[DEBUG]";
        break;
      case LogLevel::INFO:
        level_str = "[INFO]";
        break;
      case LogLevel::WARN:
        level_str = "[WARN]";
        break;
      case LogLevel::ERROR:
        level_str = "[ERROR]";
        break;
      }
      std::cerr << level_str << " " << message << std::endl;
    }
  }

  void DDSM115DriverClient::close_port()
  {
    reading_ = false;

    // IOコンテキストを安全に停止
    if (!io_context_.stopped())
    {
      io_context_.stop();
    }

    if (io_thread_.joinable())
    {
      io_thread_.join();
    }

    if (serial_port_.is_open())
    {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }
  }

  bool DDSM115DriverClient::init_port(const std::string &port_name)
  {
    try
    {
      this->port_name_ = port_name;
      this->baud_rate_ = BAUD_RATE;
      serial_port_.open(this->port_name_);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(this->baud_rate_));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

      // 非同期読み取り開始
      reading_ = true;
      start_async_read();

      // io_contextを別スレッドで実行
      if (!io_thread_.joinable())
      {
        io_thread_ = std::thread([this]()
                                 { 
        // restartが必要なケースに対応
        if (io_context_.stopped()) io_context_.restart();
        io_context_.run(); });
      }
    }
    catch (const std::exception &e)
    {
      std::stringstream ss;
      ss << "Failed to open serial port " << port_name << ": " << e.what();
      log(LogLevel::ERROR, ss.str());
      return false;
    }
    return true;
  }

  bool DDSM115DriverClient::reinitialize_port()
  {
    log(LogLevel::WARN, "Attempting to reinitialize serial port " + port_name_);

    // 一度閉じてから再オープン
    close_port();

    // 少し待つ（デバイス側のリセット待ち）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return init_port(port_name_);
  }

  void DDSM115DriverClient::send_mode_command(uint8_t motor_id, ControlLoopModes mode)
  {

    std::vector<uint8_t> data;
    data.reserve(10);

    data.push_back(motor_id);
    data.push_back(0xA0);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(static_cast<uint8_t>(mode));

    try
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
    }
    catch (const std::exception &e)
    {
      std::stringstream ss;
      ss << "Communication error with motor " << static_cast<int>(motor_id) << ": " << e.what();
      log(LogLevel::ERROR, ss.str());
      reinitialize_port();
    }

    std::stringstream ss;
    ss << "Sent mode command to motor " << static_cast<int>(motor_id) << ": mode " << static_cast<int>(mode);
    log(LogLevel::DEBUG, ss.str());
  }

  bool DDSM115DriverClient::send_current_command(uint8_t motor_id, double current)
  {
    std::vector<uint8_t> data;
    data.reserve(10);

    data.push_back(motor_id);
    data.push_back(0x64);
    uint16_t val_u16 = std::bit_cast<uint16_t>(static_cast<int16_t>(std::clamp(std::round(current * (32767.0 / 8.0)), -32767.0, 32767.0)));
    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(calc_crc8_maxim(data));
    
    std::stringstream ss;
    ss << "Current command for motor " << static_cast<int>(motor_id) << ": current=" << current << ", val_u16=" << val_u16;
    log(LogLevel::DEBUG, ss.str());

    return send_rotate_command(data, motor_id);
  }

  bool DDSM115DriverClient::send_velocity_command(uint8_t motor_id, double rpm, bool brake)
  {
    std::vector<uint8_t> data;
    data.reserve(10);

    data.push_back(motor_id);
    data.push_back(0x64);
    uint16_t val_u16 = std::bit_cast<uint16_t>(static_cast<int16_t>(std::clamp(std::round(rpm), -330.0, 330.0)));
    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(brake ? 0xFF : 0x00);
    data.push_back(0x00);
    data.push_back(calc_crc8_maxim(data));

    return send_rotate_command(data, motor_id);
  }

  bool DDSM115DriverClient::send_position_command(uint8_t motor_id, double position)
  {
    std::vector<uint8_t> data;
    data.reserve(10);

    data.push_back(motor_id);
    data.push_back(0x64);
    uint16_t val_u16 = std::bit_cast<uint16_t>(static_cast<int16_t>(std::clamp(std::round((position - 180.0) * (32767.0 / 180.0)), -32767.0, 32767.0)));
    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(calc_crc8_maxim(data));

    std::stringstream ss;
    ss << "Position command for motor " << static_cast<int>(motor_id) << ": position=" << position << ", val_u16=" << val_u16;
    log(LogLevel::DEBUG, ss.str());

    return send_rotate_command(data, motor_id);
  }

  bool DDSM115DriverClient::send_rotate_command(std::vector<uint8_t> &data, uint8_t motor_id)
  {
    try
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
      std::stringstream ss;
      ss << "Sent command to motor " << static_cast<int>(motor_id) << ", mode " << static_cast<int>(data[1]);
      log(LogLevel::DEBUG, ss.str());
      return true;
    }
    catch (const std::exception &e)
    {
      std::stringstream ss;
      ss << "Communication error with motor " << static_cast<int>(motor_id) << ": " << e.what();
      log(LogLevel::ERROR, ss.str());
      reinitialize_port();
      return false;
    }
  }
  void DDSM115DriverClient::start_async_read()
  {
    if (!reading_)
      return;
    serial_port_.async_read_some(
        boost::asio::buffer(read_buf_), [this](boost::system::error_code ec, std::size_t length)
        {
        if (!ec && length > 0) {
          {
            buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
            parse_buffer(); // ロック内でパースする
          }
        } else if (ec != boost::asio::error::operation_aborted) {
            std::stringstream ss;
            ss << "Read error: " << ec.message();
            log(LogLevel::DEBUG, ss.str());
        }
        if (reading_) start_async_read(); });
  }

  void DDSM115DriverClient::parse_buffer()
  {
    // 10バイトパケット単位でチェック
    while (buffer_.size() >= 10)
    {

      size_t pos = 0;
      bool found = false;
      for (; pos <= buffer_.size() - 10; ++pos)
      {
        // 先頭バイト(id)が1～4の範囲かチェック
        if (buffer_[pos] >= 1 && buffer_[pos] <= 4)
        {
          // CRC8チェック
          std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
          uint8_t crc_calculated = calc_crc8_maxim(packet_data);
          uint8_t crc_received = buffer_[pos + 9];
          if (crc_calculated == crc_received)
          {
            found = true;
            break; // 正しいパケット発見
          }
        }
      }

      if (pos > 0)
      {
        // 不正な先頭バイトがあれば捨てる
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
      }

      if (!found || buffer_.size() < 10)
        break;

      // 10バイトパケット取り出し
      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);

      // パースして保存
      process_feedback_packet(packet);

      // パケット消費
      buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
    }
  }

  void DDSM115DriverClient::process_feedback_packet(const std::vector<uint8_t> &packet)
  {
    uint8_t motor_id = packet[0];

    int16_t velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];

    // モーターIDと速度のみを出力
    std::stringstream ss;
    ss << "Motor ID: " << static_cast<int>(motor_id) << ", Velocity: " << velocity;
    log(LogLevel::DEBUG, ss.str());

    // コールバックが設定されていれば呼び出す
    if (feedback_callback_)
    {
      feedback_callback_(packet);
    }
  }
} // namespace ddsm115_ros2_driver
