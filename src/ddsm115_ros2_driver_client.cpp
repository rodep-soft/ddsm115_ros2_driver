#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

#include <bit>
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>

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
  close_port();
}

void DDSM115DriverClient::close_port() {
  reading_ = false;
  
  // IOコンテキストを安全に停止
  if (!io_context_.stopped()) {
    io_context_.stop();
  }
  
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  if (serial_port_.is_open()) {
    boost::system::error_code ec;
    serial_port_.close(ec);
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
    if (!io_thread_.joinable()) {
      io_thread_ = std::thread([this]() { 
        // restartが必要なケースに対応
        if (io_context_.stopped()) io_context_.restart();
        io_context_.run(); 
      });
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
    return false;
  }
  return true;
}

bool DDSM115DriverClient::reinitialize_port() {
  RCLCPP_WARN(logger_, "Attempting to reinitialize serial port %s", port_name_.c_str());
  
  // 一度閉じてから再オープン
  close_port();
  
  // 少し待つ（デバイス側のリセット待ち）
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return init_port(port_name_, baud_rate_);
}

void DDSM115DriverClient::send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake) {
  std::vector<uint8_t> data;
  data.reserve(10); 

  data.push_back(motor_id);
  data.push_back(0x64);
  uint16_t val_u16 = std::bit_cast<uint16_t>(rpm);
  data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));
  data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(brake ? 0xFF : 0x00);
  data.push_back(0x00);
  data.push_back(calc_crc8_maxim(data));

  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_motor_id_ = 0; 
    }
    // コマンド送信
    boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));

    // フィードバック待ち（参考コードのアプローチ）
    wait_for_feedback_response(motor_id, 20);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Communication error with motor %d: %s", motor_id, e.what());
    reinitialize_port();
  }
}

void DDSM115DriverClient::start_async_read() {
  if (!reading_) return;
  serial_port_.async_read_some(
      boost::asio::buffer(read_buf_), [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
            parse_buffer(); // ロック内でパースする
          }
        } else if (ec != boost::asio::error::operation_aborted) {
            RCLCPP_DEBUG(logger_, "Read error: %s", ec.message().c_str());
        }
        if (reading_) start_async_read();
      });
}

void DDSM115DriverClient::parse_buffer() {
  // 10バイトパケット単位でチェック
  while (buffer_.size() >= 10) {
    
    size_t pos = 0;
    bool found = false;
    for (; pos <= buffer_.size() - 10; ++pos) {
      // 先頭バイト(id)が1～4の範囲かチェック
      if (buffer_[pos] >= 1 && buffer_[pos] <= 4) {
        // CRC8チェック
        std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
        uint8_t crc_calculated = calc_crc8_maxim(packet_data);
        uint8_t crc_received = buffer_[pos + 9];
        if (crc_calculated == crc_received) {
          found = true;
          break;  // 正しいパケット発見
        }
      }
    }

    if (pos > 0) {
      // 不正な先頭バイトがあれば捨てる
      buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
    }

    if (!found || buffer_.size() < 10) break;

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
  
  int16_t velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];
  

  // モーターIDと速度のみを出力
  RCLCPP_DEBUG(logger_, "Motor ID: %d, Velocity: %d", motor_id, velocity);

  // フィードバック受信を記録
  feedback_received_time_ = std::chrono::steady_clock::now();
  last_motor_id_ = motor_id;

  cv_.notify_all(); 
  // コールバックが設定されていれば呼び出す
  if(feedback_callback_) {
    feedback_callback_(packet);
  }
}

void DDSM115DriverClient::wait_for_feedback_response(uint8_t motor_id, int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_); // Condition Variableには unique_lock が必要らしい

  // 述語（Predicate）付き wait_for:
  // 「タイムアウトした」 または 「last_motor_id_ が期待値になった」 まで待機する
  bool received = cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
    [this, motor_id] { 
      return last_motor_id_ == motor_id; 
    });

  if (received) {
    RCLCPP_DEBUG(logger_, "Motor %d feedback received.", motor_id);
  } else {
    RCLCPP_DEBUG(logger_, "Motor %d feedback timeout.", motor_id);
  }
}