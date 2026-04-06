/**
Software License Agreement (BSD)
\file      relay_controller.cpp
\authors   Relay Controller Node
\copyright Copyright (c) 2024, All rights reserved.
*/

#include "create_driver/relay_controller.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

RelayController::RelayController()
: Node("relay_controller"),
  serial_fd_(-1),
  connected_(false),
  status_publish_rate_(1.0)
{
  // Initialize relay states
  for (int i = 0; i < 4; i++) {
    relay_states_[i] = false;
  }
  
  // Get parameters
  dev_ = declare_parameter<std::string>("dev", "/dev/ttyUSB1");
  baud_ = declare_parameter<int>("baud", 115200);
  status_publish_rate_ = declare_parameter<double>("status_publish_rate", 1.0);

  RCLCPP_INFO_STREAM(get_logger(), "[RELAY] Using device: " << dev_ << " at " << baud_ << " baud");

  // Connect to serial device
  if (!connectSerial()) {
    RCLCPP_FATAL(get_logger(), "[RELAY] Failed to establish serial connection.");
    // Don't shutdown here - let main() handle it properly
    // The node will exit naturally when main() checks the connection
    return;
  }

  RCLCPP_INFO(get_logger(), "[RELAY] Connection established.");

  // Setup relay status message
  relay_status_msg_.data.resize(4);
  for (int i = 0; i < 4; i++) {
    relay_status_msg_.data[i] = 0;
  }

  // Setup subscribers
  relay1_sub_ = create_subscription<std_msgs::msg::Bool>(
    "relay1", 10, std::bind(&RelayController::relay1Callback, this, std::placeholders::_1));
  
  relay2_sub_ = create_subscription<std_msgs::msg::Bool>(
    "relay2", 10, std::bind(&RelayController::relay2Callback, this, std::placeholders::_1));
  
  relay3_sub_ = create_subscription<std_msgs::msg::Bool>(
    "relay3", 10, std::bind(&RelayController::relay3Callback, this, std::placeholders::_1));
  
  relay4_sub_ = create_subscription<std_msgs::msg::Bool>(
    "relay4", 10, std::bind(&RelayController::relay4Callback, this, std::placeholders::_1));
  
  relay_all_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "relay_all", 10, std::bind(&RelayController::relayAllCallback, this, std::placeholders::_1));
  
  relay_command_sub_ = create_subscription<std_msgs::msg::String>(
    "relay_command", 10, std::bind(&RelayController::relayCommandCallback, this, std::placeholders::_1));

  // Setup publishers
  relay_status_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("relay_status", 10);
  relay_feedback_pub_ = create_publisher<std_msgs::msg::String>("relay_feedback", 10);

  // Setup status timer
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_publish_rate_));
  status_timer_ = create_wall_timer(timer_period, std::bind(&RelayController::publishStatus, this));

  RCLCPP_INFO(get_logger(), "[RELAY] Ready.");
}

RelayController::~RelayController()
{
  RCLCPP_INFO(get_logger(), "[RELAY] Destruct sequence initiated.");
  disconnectSerial();
}

bool RelayController::connectSerial()
{
  serial_fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "[RELAY] Failed to open serial port: %s", dev_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[RELAY] Failed to get serial attributes");
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_) {
    case 115200: speed = B115200; break;
    case 57600:  speed = B57600;  break;
    case 38400:  speed = B38400;  break;
    case 19200:  speed = B19200;  break;
    case 9600:   speed = B9600;   break;
    default:
      RCLCPP_ERROR(get_logger(), "[RELAY] Unsupported baud rate: %d", baud_);
      return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Configure serial port: 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[RELAY] Failed to set serial attributes");
    return false;
  }

  connected_ = true;
  
  // Wait for Arduino to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  // Read initial messages from Arduino
  readSerialData();
  
  return true;
}

void RelayController::disconnectSerial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
}

bool RelayController::sendCommand(const std::string& command)
{
  if (!connected_ || serial_fd_ < 0) {
    RCLCPP_WARN(get_logger(), "[RELAY] Not connected, cannot send command: %s", command.c_str());
    return false;
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  std::string full_command = command + "\n";
  ssize_t written = write(serial_fd_, full_command.c_str(), full_command.length());
  bool success = written == static_cast<ssize_t>(full_command.length());
  
  if (success) {
    RCLCPP_INFO(get_logger(), "[RELAY] Sent command: %s", command.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "[RELAY] Failed to send command: %s (written: %ld, expected: %zu)", 
                command.c_str(), written, full_command.length());
  }
  
  return success;
}

void RelayController::readSerialData()
{
  if (!connected_ || serial_fd_ < 0) {
    return;
  }

  char buffer[256];
  std::string data;
  
  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t n = read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (n > 0) {
    buffer[n] = '\0';
    data = std::string(buffer);
    
    // Parse responses line by line
    std::stringstream ss(data);
    std::string line;
    while (std::getline(ss, line)) {
      if (!line.empty()) {
        parseResponse(line);
      }
    }
  }
}

void RelayController::parseResponse(const std::string& response)
{
  RCLCPP_INFO(get_logger(), "[RELAY] Received: %s", response.c_str());
  
  if (response.find("RELAY_CONTROLLER_READY") != std::string::npos) {
    RCLCPP_INFO(get_logger(), "[RELAY] Arduino ready");
    return;
  }
  
  if (response.find("RELAY") != std::string::npos && response.find(":") != std::string::npos) {
    // Parse relay status response: RELAY1:ON or RELAY1:OFF
    size_t colon_pos = response.find(":");
    if (colon_pos != std::string::npos) {
      std::string relay_part = response.substr(0, colon_pos);
      std::string state_part = response.substr(colon_pos + 1);
      
      // Extract relay number
      if (relay_part.length() >= 6 && relay_part.substr(0, 5) == "RELAY") {
        int relay_num = std::stoi(relay_part.substr(5, 1));
        bool state = (state_part == "ON");
        
        if (relay_num >= 1 && relay_num <= 4) {
          std::lock_guard<std::mutex> lock(relay_mutex_);
          relay_states_[relay_num - 1] = state;
        }
      }
    }
  }
  else if (response.find("STATUS:") != std::string::npos) {
    // Parse status response: STATUS:1,0,1,0
    size_t colon_pos = response.find(":");
    if (colon_pos != std::string::npos) {
      std::string status_data = response.substr(colon_pos + 1);
      std::stringstream ss(status_data);
      std::string item;
      int i = 0;
      
      std::lock_guard<std::mutex> lock(relay_mutex_);
      while (std::getline(ss, item, ',') && i < 4) {
        relay_states_[i] = (item == "1");
        i++;
      }
    }
  }
  else if (response.find("ERROR:") != std::string::npos) {
    RCLCPP_ERROR(get_logger(), "[RELAY] Arduino error: %s", response.c_str());
  }
  
  // Publish feedback
  feedback_msg_.data = response;
  relay_feedback_pub_->publish(feedback_msg_);
}

void RelayController::setRelay(int relayNum, bool state)
{
  if (relayNum < 1 || relayNum > 4) {
    RCLCPP_ERROR(get_logger(), "[RELAY] Invalid relay number: %d", relayNum);
    return;
  }
  
  std::string command = "RELAY" + std::to_string(relayNum) + (state ? "_ON" : "_OFF");
  sendCommand(command);
  
  // Add small delay to prevent command concatenation
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void RelayController::publishStatus()
{
  readSerialData();
  
  std::lock_guard<std::mutex> lock(relay_mutex_);
  for (int i = 0; i < 4; i++) {
    relay_status_msg_.data[i] = relay_states_[i] ? 1 : 0;
  }
  
  relay_status_pub_->publish(relay_status_msg_);
}

void RelayController::relay1Callback(std_msgs::msg::Bool::UniquePtr msg)
{
  setRelay(1, msg->data);
}

void RelayController::relay2Callback(std_msgs::msg::Bool::UniquePtr msg)
{
  setRelay(2, msg->data);
}

void RelayController::relay3Callback(std_msgs::msg::Bool::UniquePtr msg)
{
  setRelay(3, msg->data);
}

void RelayController::relay4Callback(std_msgs::msg::Bool::UniquePtr msg)
{
  setRelay(4, msg->data);
}

void RelayController::relayAllCallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg)
{
  if (msg->data.size() >= 4) {
    for (int i = 0; i < 4; i++) {
      setRelay(i + 1, msg->data[i] != 0);
      // Add small delay between commands to prevent concatenation
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } else {
    RCLCPP_ERROR(get_logger(), "[RELAY] Invalid relay_all message size: %zu", msg->data.size());
  }
}

void RelayController::relayCommandCallback(std_msgs::msg::String::UniquePtr msg)
{
  sendCommand(msg->data);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto relay_controller = std::make_shared<RelayController>();
  
  // Check if connection was established
  if (!relay_controller->isConnected()) {
    RCLCPP_FATAL(relay_controller->get_logger(), "[RELAY] Failed to establish serial connection. Exiting.");
    rclcpp::shutdown();
    return 1;
  }
  
  try {
    rclcpp::spin(relay_controller);
  } catch (std::runtime_error& ex) {
    RCLCPP_FATAL_STREAM(relay_controller->get_logger(), "[RELAY] Runtime error: " << ex.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
