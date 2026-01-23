#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

#include <bit>
#include <cmath>
#include <thread>

uint8_t DDSM115DriverClient::calc_crc8_maxim(const std::vector<uint8_t>& data) {
  uint8_t crc = 0x00;  // 初期値  (一般的なMaxim CRCの標準)

  const uint8_t reflected_polynomial = 0x8C;

  // データバイトを一つずつ処理
  for (size_t i = 0; i < data.size(); i++) {  // DATA[0]~DATA[8]まで、合計9バイト
    crc ^= data[i];                           // 現在のバイトとCRCレジスタをXOR

    // 各バイトの8ビットを処理 (LSB First)
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {                           // 最下位ビットが1の場合
        crc = (crc >> 1) ^ reflected_polynomial;  // 右シフトして多項式とXOR
      } else {
        crc >>= 1;  // 最下位ビットが0の場合、単に右シフト
      }
    }
  }
  return crc;
}

DDSM115DriverClient::DDSM115DriverClient(rclcpp::Logger logger, std::function<void(const std::vector<uint8_t>&)> feedback_callback)
    : serial_port_(io_context_), logger_(logger), buffer_(), reading_(false), feedback_callback_(feedback_callback) {}

DDSM115DriverClient::~DDSM115DriverClient() {
  if (reading_) {
    reading_ = false;
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }
}

bool DDSM115DriverClient::init_port(const std::string& port_name, int baud_rate) {
  try {
    this->port_name_ = port_name;
    this->baud_rate_ = baud_rate;
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
    io_thread_ = std::thread([this]() { io_context_.run(); });

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
    return false;
  }
  return true;
}

bool DDSM115DriverClient::reinitialize_port() {
  reading_ = false;
  io_context_.stop();
  if(io_thread_.joinable()) {
    io_thread_.join();
  }
  try {
    if(serial_port_.is_open()) {
      serial_port_.close();
    }
    serial_port_.open(this->port_name_);

    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(this->baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    serial_port_.set_option(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(
        boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));    

    io_context_.restart();
    // 非同期読み取り開始
    reading_ = true;
    start_async_read();

    // io_contextを別スレッドで実行
    io_thread_ = std::thread([this]() { io_context_.run(); });
  
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name_.c_str(), e.what());
    return false;
  }

  RCLCPP_INFO(logger_, "Reinitialized serial port %s", port_name_.c_str());
  return true;
}

void DDSM115DriverClient::send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake) {
  std::vector<uint8_t> data;

  data.push_back(static_cast<uint8_t>(motor_id & 0xFF));
  data.push_back(0x64);                             // DDSM115 velocity command
  uint16_t val_u16 = std::bit_cast<uint16_t>(rpm);  // 符号付きを符号なしに変換

  data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));  // Highバイト
  data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));         // Lowバイト

  data.push_back(0x00);  // Acceleration time (0 = default)
  data.push_back(0x00);  // Reserved
  data.push_back(0x00);  // Reserved
  if (brake) {
    data.push_back(0xFF);  // Brake command
  } else {
    data.push_back(0x00);
  }
  data.push_back(0x00);                   // Reserved
  data.push_back(calc_crc8_maxim(data));  // CRC8

  try {
    // コマンド送信
    boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));

    // フィードバック待ち（参考コードのアプローチ）
    wait_for_feedback_response(motor_id);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to communicate with motor %d: %s", motor_id, e.what());
    if (!reinitialize_port()) {
      RCLCPP_ERROR(logger_, "Failed to reinitialize port: %s", port_name_.c_str());
      char temp = port_name_[port_name_.size() - 1];
      port_name_[port_name_.size() - 1] = '1';
      if (!reinitialize_port()) {
        RCLCPP_ERROR(logger_, "Failed to reinitialize port: %s", port_name_.c_str());
        port_name_[port_name_.size() - 1] = temp;
      }
    }
  }
}

void DDSM115DriverClient::clear_serial_buffer() {
  try {
    // バッファクリアのため短時間で複数回読み取り試行
    for (int i = 0; i < 5; ++i) {
      try {
        std::vector<uint8_t> buffer(64);
        boost::system::error_code error;

        std::size_t bytes_read = boost::asio::read(serial_port_, boost::asio::buffer(buffer),
                                                   boost::asio::transfer_at_least(1), error);

        if (!error && bytes_read > 0) {
          RCLCPP_DEBUG(logger_, "Cleared %zu bytes from serial buffer", bytes_read);
        } else {
          break;  // 読み取るデータがない場合は終了
        }
      } catch (const std::exception&) {
        break;  // エラーまたはタイムアウトで終了
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(logger_, "Error clearing serial buffer: %s", e.what());
  }
}

void DDSM115DriverClient::start_async_read() {
  if (!reading_) return;
  serial_port_.async_read_some(
      boost::asio::buffer(read_buf_), [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
          parse_buffer();
        } else {
          if (ec) {
            RCLCPP_DEBUG(logger_, "Read error: %s", ec.message().c_str());
          }
        }
        start_async_read();
      });
}

void DDSM115DriverClient::parse_buffer() {
  // 10バイトパケット単位でチェック
  while (buffer_.size() >= 10) {
    // パケット開始候補を探す(IDは1~4の範囲)
    size_t pos = 0;
    for (; pos <= buffer_.size() - 10; ++pos) {
      if (buffer_[pos] >= 1 && buffer_[pos] <= 4) {
        // CRC8チェック
        std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
        uint8_t crc_calculated = calc_crc8_maxim(packet_data);
        uint8_t crc_received = buffer_[pos + 9];
        if (crc_calculated == crc_received) {
          break;  // 正しいパケット発見
        }
      }
    }

    if (pos > 0) {
      // 不正な先頭バイトがあれば捨てる
      buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
    }

    if (buffer_.size() < 10) break;

    // 10バイトパケット取り出し
    std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);

    // パースして保存
    process_feedback_packet(packet);

    // パケット消費
    buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
  }
}

void DDSM115DriverClient::process_feedback_packet(const std::vector<uint8_t>& packet) {
  uint8_t motor_id = packet[0];
  // uint8_t mode_value = packet[1];
  // int16_t torque_current = (static_cast<int16_t>(packet[2]) << 8) | packet[3];
  int16_t velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];
  // uint16_t position = (static_cast<uint16_t>(packet[6]) << 8) | packet[7];
  // uint8_t error_code = packet[8];

  // モーターIDと速度のみを出力
  RCLCPP_DEBUG(logger_, "Motor ID: %d, Velocity: %d", motor_id, velocity);

  // フィードバック受信を記録
  feedback_received_time_ = std::chrono::steady_clock::now();
  last_motor_id_ = motor_id;

  // コールバックが設定されていれば呼び出す
  if(feedback_callback_) {
    feedback_callback_(packet);
  }
}

void DDSM115DriverClient::wait_for_feedback_response(uint8_t motor_id, int timeout_ms) {
  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::milliseconds(timeout_ms);

  // フィードバック受信をリセット
  last_motor_id_ = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    // 少し待機してフィードバック受信をチェック
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (last_motor_id_ == motor_id) {
      // 正しいモーターIDからフィードバック受信
      RCLCPP_DEBUG(logger_, "Motor %d feedback received successfully", motor_id);
      return;
    }
  }

  // タイムアウト - でも動作は継続
  RCLCPP_DEBUG(logger_, "Motor %d feedback timeout, but continuing", motor_id);
}