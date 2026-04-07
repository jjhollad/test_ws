/**
Software License Agreement (BSD)
\file      generic_motor_driver.h
\authors   Adapted from create_driver
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_
#define GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

static const double COVARIANCE[36] = {1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,  // NOLINT(whitespace/braces)
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5,
                                      0.0,  0.0,  1e-5, 0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  1e-5, 0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  1e-5, 0.0,
                                      1e-5, 1e-5, 0.0,  0.0,  0.0,  1e-5};

struct MotorData {
  double total_encoder[2] = {0.0, 0.0};  // Total encoder counts for M1-M2 (left and right wheels) - from MAll
  double realtime_encoder[2] = {0.0, 0.0};  // Real-time encoder counts (10ms) for M1-M2 - from MTEP
  double speed[2] = {0.0, 0.0};  // Speed for M1-M2 - from MSPD (not used for odometry, kept for compatibility)
  rclcpp::Time timestamp;
};

class GenericMotorDriver : public rclcpp::Node
{
private:
  // Serial communication
  std::string dev_;
  int baud_;
  int serial_fd_;
  std::atomic<bool> connected_;
  std::mutex serial_mutex_;
  
  // Motor data
  MotorData motor_data_;
  std::mutex motor_data_mutex_;
  bool total_encoder_initialized_{false};
  
  // Odometry calculation
  double wheel_base_;  // Distance between left and right wheels
  double wheel_radius_;  // Wheel radius

  double motor_gear_ratio_;  // Motor internal gear ratio (e.g., 50.0 for 1:50)
  double belt_drive_ratio_;  // Belt drive ratio (e.g., 2.0 for 1:2)
  double total_reduction_ratio_;  // Total reduction ratio
  double x_, y_, theta_;  // Robot pose
  double last_left_encoder_, last_right_encoder_;
  rclcpp::Time last_odom_time_;
  bool encoders_initialized_;  // Track if encoders have been initialized
  
  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_speeds_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_rx_pub_;  // Raw serial data received
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_tx_pub_;  // Raw serial commands sent
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv_;
  
  rclcpp::TimerBase::SharedPtr loop_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  diagnostic_updater::Updater diagnostics_;
  
  // Messages
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped tf_odom_;
  sensor_msgs::msg::JointState joint_state_msg_;
  rclcpp::Time last_cmd_vel_time_;
  std_msgs::msg::Float32 float32_msg_;
  
  // Cached motor speeds (updated by cmd_vel, sent in update loop)
  double cached_left_motor_speed_;
  double cached_right_motor_speed_;
  std::mutex cmd_vel_mutex_;
  
  // ROS parameters
  std::string base_frame_;
  std::string odom_frame_;
  rclcpp::Duration latch_duration_;
  double loop_hz_;
  bool publish_tf_;
  bool is_running_slowly_;
  double max_motor_speed_;  // Maximum motor speed (-100 to 100)
  std::vector<std::string> joint_names_;  // Joint names for joint_state (must match URDF)
  bool invert_left_encoder_;  // Invert left encoder direction
  bool invert_right_encoder_;  // Invert right encoder direction
  bool invert_left_motor_;  // Invert left motor direction (for backwards motors)
  bool invert_right_motor_;  // Invert right motor direction (for backwards motors)
  /// First-order low-pass on cmd_vel (1.0 = off). Reduces joystick noise / deadzone dither.
  double cmd_vel_filter_alpha_;
  double filtered_cmd_lin_x_;
  double filtered_cmd_ang_z_;
  bool cmd_vel_filter_initialized_;
  double joint_state_left_sign_;   // RViz-only sign for left joint_state (+1 or -1)
  double joint_state_right_sign_;  // RViz-only sign for right joint_state (+1 or -1)
  bool apply_gear_reduction_;  // If true, divide encoder by gear ratio (encoder = motor counts). If false, encoder already in wheel rotations.
  double encoder_reduction_factor_;  // Additional reduction factor to apply to encoder values (multiplies the reduction)
  
  // MSPD data tracking
  bool mspd_received_;  // Track if MSPD data has been received
  bool upload_command_sent_;  // Track if upload command has been sent (only send once)
  rclcpp::Time mspd_check_start_time_;  // Time when we start checking for MSPD
  
  // Serial communication methods
  bool connectSerial();
  void disconnectSerial();
  bool sendCommand(const std::string& command);
  void readSerialData();
  void parseMotorData(const std::string& data);
  
  // Motor control methods
  void sendMotorSpeeds(double m1, double m2, double m3, double m4);
  void updateOdometry();
  
  // ROS2 callbacks
  void cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg);
  void resetOdometryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response);
  
  // Publishing methods
  void publishOdom();
  void publishJointState();
  bool update();
  
  // Diagnostic methods
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

public:
  GenericMotorDriver();
  ~GenericMotorDriver();
  virtual void spinOnce();
};

#endif  // GENERIC_MOTOR_DRIVER__GENERIC_MOTOR_DRIVER_H_
