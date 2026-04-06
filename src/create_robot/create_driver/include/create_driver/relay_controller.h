/**
Software License Agreement (BSD)
\file      relay_controller.h
\authors   Relay Controller Node
\copyright Copyright (c) 2024, All rights reserved.
*/

#ifndef RELAY_CONTROLLER__RELAY_CONTROLLER_H_
#define RELAY_CONTROLLER__RELAY_CONTROLLER_H_

#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

class RelayController : public rclcpp::Node
{
private:
  // Serial communication
  std::string dev_;
  int baud_;
  int serial_fd_;
  std::atomic<bool> connected_;
  std::mutex serial_mutex_;
  
  // Relay states
  bool relay_states_[4];
  std::mutex relay_mutex_;
  
  // ROS2 interfaces
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay1_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay2_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay3_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relay4_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr relay_all_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr relay_command_sub_;
  
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr relay_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_feedback_pub_;
  
  rclcpp::TimerBase::SharedPtr status_timer_;
  
  // Messages
  std_msgs::msg::UInt8MultiArray relay_status_msg_;
  std_msgs::msg::String feedback_msg_;
  
  // ROS parameters
  double status_publish_rate_;
  
  // Serial communication methods
  bool connectSerial();
  void disconnectSerial();
  bool sendCommand(const std::string& command);
  void readSerialData();
  void parseResponse(const std::string& response);
  
  // Relay control methods
  void setRelay(int relayNum, bool state);
  void publishStatus();
  
  // ROS2 callbacks
  void relay1Callback(std_msgs::msg::Bool::UniquePtr msg);
  void relay2Callback(std_msgs::msg::Bool::UniquePtr msg);
  void relay3Callback(std_msgs::msg::Bool::UniquePtr msg);
  void relay4Callback(std_msgs::msg::Bool::UniquePtr msg);
  void relayAllCallback(std_msgs::msg::UInt8MultiArray::UniquePtr msg);
  void relayCommandCallback(std_msgs::msg::String::UniquePtr msg);

public:
  RelayController();
  ~RelayController();
  bool isConnected() const { return connected_; }
};

#endif  // RELAY_CONTROLLER__RELAY_CONTROLLER_H_
