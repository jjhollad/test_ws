/**
Software License Agreement (BSD)
\file      generic_motor_driver.cpp
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

#include "create_driver/generic_motor_driver.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <exception>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GenericMotorDriver::GenericMotorDriver()
: Node("generic_motor_driver"),
  serial_fd_(-1),
  connected_(false),
  wheel_base_(0.61),  // Default 30cm wheelbase
  wheel_radius_(0.112),  // Default 5cm wheel radius
  motor_gear_ratio_(90.0),  // Default no gear reduction
  belt_drive_ratio_(6.4),  // Default no belt drive
  total_reduction_ratio_(1.0),  // Will be calculated
  x_(0.0), y_(0.0), theta_(0.0),
  last_left_encoder_(0.0), last_right_encoder_(0.0),
  last_odom_time_(),  // Will be set to now() in constructor body to ensure correct clock type
  encoders_initialized_(false),
  last_cmd_vel_time_(),  // Will be set to now() in constructor body to ensure correct clock type
  latch_duration_(std::chrono::nanoseconds{0}),
  is_running_slowly_(false),
  cached_left_motor_speed_(0.0),
  cached_right_motor_speed_(0.0),
  tf_broadcaster_(this),
  diagnostics_(this)
{
  // Get parameters
  dev_ = declare_parameter<std::string>("dev", "/dev/motor_controller");
  baud_ = declare_parameter<int>("baud", 115200);
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  wheel_base_ = declare_parameter<double>("wheel_base", 0.61);
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.112);
  motor_gear_ratio_ = declare_parameter<double>("motor_gear_ratio", 90.0);
  belt_drive_ratio_ = declare_parameter<double>("belt_drive_ratio", 6.4);
  latch_duration_ = rclcpp::Duration::from_seconds(declare_parameter<double>("latch_cmd_duration", 0.2));
  loop_hz_ = declare_parameter<double>("loop_hz", 20.0);  // Default 20 Hz for lower latency
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  max_motor_speed_ = declare_parameter<double>("max_motor_speed", 980.0);
  invert_left_encoder_ = declare_parameter<bool>("invert_left_encoder", false);
  invert_right_encoder_ = declare_parameter<bool>("invert_right_encoder", true);
  invert_left_motor_ = declare_parameter<bool>("invert_left_motor", false);
  invert_right_motor_ = declare_parameter<bool>("invert_right_motor", true);
  // RViz-only: multiply published /joint_states position & velocity for each wheel (+1 or -1).
  // Does not affect odometry; use to match URDF wheel axis vs encoder chirality.
  joint_state_left_sign_ = declare_parameter<double>("joint_state_left_sign", -1.0);
  joint_state_right_sign_ = declare_parameter<double>("joint_state_right_sign", 1.0);
  // Normalize to exactly ±1 so any positive/negative parameter value maps to one direction.
  joint_state_left_sign_ = (joint_state_left_sign_ >= 0.0) ? 1.0 : -1.0;
  joint_state_right_sign_ = (joint_state_right_sign_ >= 0.0) ? 1.0 : -1.0;
  
  // Joint names (default to Create 2 URDF joint names)
  declare_parameter<std::vector<std::string>>("joint_names", 
    std::vector<std::string>{"left_wheel_joint", "right_wheel_joint"});
  joint_names_ = get_parameter("joint_names").as_string_array();
  
  // Ensure we have at least 2 joint names (for left and right wheels)
  if (joint_names_.size() < 2) {
    RCLCPP_WARN(get_logger(), "Only %zu joint names provided, need at least 2. Using defaults.", joint_names_.size());
    joint_names_ = {"left_wheel_joint", "right_wheel_joint"};
  }
  // Resize to 2 if needed
  if (joint_names_.size() > 2) {
    RCLCPP_WARN(get_logger(), "More than 2 joint names provided, using only first 2.");
    joint_names_.resize(2);
  }
  
  // Calculate total reduction ratio
  total_reduction_ratio_ = motor_gear_ratio_ * belt_drive_ratio_;

  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Using device: " << dev_ << " at " << baud_ << " baud");
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Motor gear ratio: " << motor_gear_ratio_);
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Belt drive ratio: " << belt_drive_ratio_);
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Total reduction ratio: " << total_reduction_ratio_);
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Encoder inversion: left=" << (invert_left_encoder_ ? "true" : "false") 
                     << ", right=" << (invert_right_encoder_ ? "true" : "false"));
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] Motor inversion: left=" << (invert_left_motor_ ? "true" : "false") 
                     << ", right=" << (invert_right_motor_ ? "true" : "false"));
  RCLCPP_INFO_STREAM(get_logger(), "[MOTOR] JointState RViz sign: left=" << joint_state_left_sign_
                     << ", right=" << joint_state_right_sign_);

  // Connect to serial device
  if (!connectSerial()) {
    RCLCPP_FATAL(get_logger(), "[MOTOR] Failed to establish serial connection.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "[MOTOR] Connection established.");
  RCLCPP_INFO(get_logger(), "[MOTOR] Waiting for encoder data from motor controller...");

  // Set frame_id's
  tf_odom_.header.frame_id = odom_frame_;
  tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  
  // Setup joint state message (joint names will be set from parameters after they're loaded)
  joint_state_msg_.name.resize(2);
  joint_state_msg_.position.resize(2);
  joint_state_msg_.velocity.resize(2);
  joint_state_msg_.effort.resize(2);
  // Joint names are set from parameters (see above)
  for (size_t i = 0; i < 2 && i < joint_names_.size(); i++) {
    joint_state_msg_.name[i] = joint_names_[i];
  }

  // Populate initial covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&GenericMotorDriver::cmdVelCallback, this, std::placeholders::_1));

  // Setup publishers (larger queue for smoother publishing)
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);
  motor_speeds_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("motor_speeds", 50);
  serial_rx_pub_ = create_publisher<std_msgs::msg::String>("serial_rx", 50);  // Raw serial data received
  serial_tx_pub_ = create_publisher<std_msgs::msg::String>("serial_tx", 50);  // Raw serial commands sent

  // Setup diagnostics
  diagnostics_.add("Serial Status", this, &GenericMotorDriver::updateSerialDiagnostics);
  diagnostics_.add("Driver Status", this, &GenericMotorDriver::updateDriverDiagnostics);
  diagnostics_.setHardwareID("Generic Motor Controller");

  // Setup update loop
  const auto loop_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_hz_));
  loop_timer_ = create_wall_timer(loop_period, std::bind(&GenericMotorDriver::spinOnce, this));

  // Initialize time variables with current time (now that Node is fully constructed)
  // Use get_clock()->now() to ensure all times use the same clock source (ROS time or system time)
  rclcpp::Time init_time = this->get_clock()->now();
  last_odom_time_ = init_time;
  last_cmd_vel_time_ = init_time;
  
  // Initialize motor_data timestamp with the same clock source
  {
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    motor_data_.timestamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  RCLCPP_INFO(get_logger(), "[MOTOR] Ready.");
}

GenericMotorDriver::~GenericMotorDriver()
{
  RCLCPP_INFO(get_logger(), "[MOTOR] Destruct sequence initiated.");
  disconnectSerial();
}

bool GenericMotorDriver::connectSerial()
{
  serial_fd_ = open(dev_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to open serial port: %s", dev_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to get serial attributes");
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
      RCLCPP_ERROR(get_logger(), "[MOTOR] Unsupported baud rate: %d", baud_);
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
  tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout (reduced for lower latency)

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "[MOTOR] Failed to set serial attributes");
    return false;
  }

  connected_ = true;
  sendCommand("$upload:1,1,1#\r\n");
  sendCommand("$mline:64#\r\n");
  sendCommand("$mphase:576#\r\n");
  sendCommand("$wdiameter:230#\r\n");
  return true;
}

void GenericMotorDriver::disconnectSerial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
}

bool GenericMotorDriver::sendCommand(const std::string& command)
{
  if (!connected_ || serial_fd_ < 0) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Not connected, cannot send command: %s", command.c_str());
    return false;
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t written = write(serial_fd_, command.c_str(), command.length());
  bool success = written == static_cast<ssize_t>(command.length());
  
  // Publish serial command sent for logging/monitoring
  if (serial_tx_pub_ && success) {
    std_msgs::msg::String tx_msg;
    tx_msg.data = command;
    serial_tx_pub_->publish(tx_msg);
  }
  
  if (success) {
    //RCLCPP_INFO(get_logger(), "[MOTOR] Sent command: %s", command.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "[MOTOR] Failed to send command: %s (written: %ld, expected: %zu)", 
                command.c_str(), written, command.length());
  }
  
  return success;
}

void GenericMotorDriver::readSerialData()
{
  if (!connected_ || serial_fd_ < 0) {
    return;
  }

  char buffer[256];
  std::string data;
  
  std::lock_guard<std::mutex> lock(serial_mutex_);
  // Hard-drop any queued RX bytes so parsing uses only freshest data.
  // This intentionally trades completeness for minimal backlog latency.
  if (tcflush(serial_fd_, TCIFLUSH) != 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[MOTOR] tcflush(TCIFLUSH) failed: %s", std::strerror(errno));
  }

  ssize_t n = read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (n > 0) {
    buffer[n] = '\0';
    data = std::string(buffer);
    
    // Parse any complete messages in the buffer
    size_t start = 0;
    while (true) {
      size_t dollar_pos = data.find('$', start);
      if (dollar_pos == std::string::npos) break;
      
      size_t hash_pos = data.find('#', dollar_pos);
      if (hash_pos == std::string::npos) break;
      
      std::string message = data.substr(dollar_pos, hash_pos - dollar_pos + 1);
      parseMotorData(message);
      
      start = hash_pos + 1;
    }
  }
}

void GenericMotorDriver::parseMotorData(const std::string& data)
{
  if (data.length() < 3) return;
  
  // Publish raw serial data for logging/monitoring
  if (serial_rx_pub_) {
    std_msgs::msg::String rx_msg;
    rx_msg.data = data;
    serial_rx_pub_->publish(rx_msg);
  }
  
  //RCLCPP_INFO(get_logger(), "[MOTOR] Received data: %s", data.c_str());
  
  std::string command = data.substr(1, 4);  // Extract command (e.g., "MAll", "MTEP", "MSPD")

  auto parse_first_two_values = [&](const std::string& payload, double (&target)[2], bool update_timestamp) {
    std::stringstream ss(payload);
    std::string item;
    int i = 0;

    while (std::getline(ss, item, ',') && i < 4) {
      if (i < 2) {  // Only store M1 and M2
        const auto first = item.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "[MOTOR] Empty numeric field in %s payload: '%s'",
                               command.c_str(), payload.c_str());
          i++;
          continue;
        }
        const auto last = item.find_last_not_of(" \t\r\n");
        const std::string token = item.substr(first, last - first + 1);

        try {
          target[i] = std::stod(token);
        } catch (const std::exception& ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "[MOTOR] Failed parsing %s token '%s' in payload '%s': %s",
                               command.c_str(), token.c_str(), payload.c_str(), ex.what());
        }
      }
      i++;
    }

    if (update_timestamp) {
      motor_data_.timestamp = this->get_clock()->now();
    }
  };
  
  if (command == "MAll") {
    // Parse total encoder data: $MAll:M1,M2,M3,M4# (only use M1 and M2)
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      parse_first_two_values(values, motor_data_.total_encoder, true);
      total_encoder_initialized_ = true;
      //RCLCPP_INFO(get_logger(), "[MOTOR] Updated total encoders: [%.1f, %.1f]", 
      //             motor_data_.total_encoder[0], motor_data_.total_encoder[1]);
    }
  }
  else if (command == "MTEP") {
    // Parse real-time encoder data: $MTEP:M1,M2,M3,M4# (only use M1 and M2)
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      parse_first_two_values(values, motor_data_.realtime_encoder, true);  // Update timestamp with encoder data
    }
  }
  else if (command == "MSPD") {
    // Parse speed data: $MSPD:M1,M2,M3,M4# (only use M1 and M2)
    size_t colon_pos = data.find(':');
    if (colon_pos != std::string::npos) {
      std::string values = data.substr(colon_pos + 1, data.length() - colon_pos - 2);
      std::lock_guard<std::mutex> lock(motor_data_mutex_);
      parse_first_two_values(values, motor_data_.speed, false);
    }
  }
}

void GenericMotorDriver::sendMotorSpeeds(double m1, double m2, double m3, double m4)
{
  (void)m3;
  (void)m4;

  // Only send M1 and M2, set M3 and M4 to 0
  std::stringstream ss;
  const int m1_i = static_cast<int>(m1);
  const int m2_i = static_cast<int>(m2);

  ss << "$spd:" << m1_i << "," << m2_i
     << ",0,0#";  // M3 and M4 always 0
  
  // Debug: Log motor speeds occasionally (reduced frequency for performance)
  static int debug_counter = 0;
  if (++debug_counter % 100 == 0) {  // Log every 100th call (about every 10 seconds at 10 Hz)
    RCLCPP_DEBUG(get_logger(), "[MOTOR] Sending speeds: M1=%.1f, M2=%.1f (invert_left=%s, invert_right=%s)",
                 m1, m2, invert_left_motor_ ? "true" : "false", invert_right_motor_ ? "true" : "false");
  }
  
  if (!sendCommand(ss.str())) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Failed to send motor command");
  }

  // Publish the exact integer $spd values we send over serial.
  // Topic: /motor_speeds  data: [M1, M2]
  if (motor_speeds_pub_) {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = m1_i;
    msg.data[1] = m2_i;
    motor_speeds_pub_->publish(msg);
  }
}

void GenericMotorDriver::updateOdometry()
{
  std::lock_guard<std::mutex> lock(motor_data_mutex_);

  // Wait for first real TOTAL encoder sample before initializing baseline.
  // MTEP can arrive first and set timestamp while total_encoder is still default 0,0.
  if (!total_encoder_initialized_) {
    return;
  }
  
  // Use motors 1 and 2 as left and right wheels for differential drive
  double left_encoder = motor_data_.total_encoder[0];
  double right_encoder = motor_data_.total_encoder[1];
  
  // Apply encoder inversion if needed (to fix spinning in circles when driving straight)
  if (invert_left_encoder_) {
    left_encoder = -left_encoder;
  }
  if (invert_right_encoder_) {
    right_encoder = -right_encoder;
  }
  
  // Initialize encoders on first reading to avoid large initial jump
  if (!encoders_initialized_) {
    last_left_encoder_ = left_encoder;
    last_right_encoder_ = right_encoder;
    encoders_initialized_ = true;
    RCLCPP_INFO(get_logger(), "[MOTOR] Initialized encoders: L=%.1f, R=%.1f (inverted: L=%s, R=%s)", 
                 left_encoder, right_encoder,
                 invert_left_encoder_ ? "yes" : "no",
                 invert_right_encoder_ ? "yes" : "no");
    last_odom_time_ = this->now();
    return;  // Skip first update to avoid large jump
  }
  
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_odom_time_).seconds();
  
  // Calculate encoder deltas
  double left_delta = left_encoder - last_left_encoder_;
  double right_delta = right_encoder - last_right_encoder_;
  
  // Check if encoders have meaningfully changed (at least 1 encoder count difference)
  // This prevents unnecessary updates when robot is stationary
  bool encoders_changed = (std::abs(left_delta) >= 1.0) || (std::abs(right_delta) >= 1.0);
  
  // Only update if encoders have changed AND sufficient time has passed.
  // Keep this small to avoid visible odom lag at 20 Hz loop rates.
  if (encoders_changed && dt > 0.01) {
    // Convert encoder counts to wheel rotations, accounting for gear ratios
    double left_wheel_rotations = left_delta / (64.0 * total_reduction_ratio_);
    double right_wheel_rotations = right_delta / (64.0 * total_reduction_ratio_);
    
    // Convert wheel rotations to linear distance
    double left_distance = left_wheel_rotations * (2.0 * M_PI * wheel_radius_);
    double right_distance = right_wheel_rotations * (2.0 * M_PI * wheel_radius_);
    
    double delta_distance = (left_distance + right_distance) / 2.0;
    // Angular velocity: positive = counter-clockwise (left wheel forward, right wheel backward)
    // Formula: (left - right) / wheel_base gives positive for counter-clockwise
    double delta_theta = (left_distance - right_distance) / wheel_base_;
    
    // Debug: Log odometry deltas occasionally (reduced frequency for performance)
    static int odom_debug_counter = 0;
    if (++odom_debug_counter % 200 == 0) {  // Every 20 seconds at 10 Hz
      RCLCPP_DEBUG(get_logger(), "[MOTOR] Odometry: delta_dist=%.4f, delta_theta=%.4f, encoder_deltas: L=%.1f, R=%.1f",
                   delta_distance, delta_theta, left_delta, right_delta);
    }
    
    x_ += delta_distance * cos(theta_);
    y_ += delta_distance * sin(theta_);
    theta_ += delta_theta;
    
    last_left_encoder_ = left_encoder;
    last_right_encoder_ = right_encoder;
    last_odom_time_ = current_time;
  }
}

void GenericMotorDriver::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg)
{
  // Convert twist to motor speeds
  // Assuming motors 1 and 2 are left and right wheels
  double linear = msg->linear.x;
  double angular = msg->angular.z;
  
  // Calculate wheel angular velocities (rad/s)
  // Note: Left motor is negated when sent, so we swap angular signs
  double left_wheel_angular_vel = (linear + angular * wheel_base_ / 2.0) / wheel_radius_;
  double right_wheel_angular_vel = (linear - angular * wheel_base_ / 2.0) / wheel_radius_;
  
  // Convert wheel angular velocity to motor controller speed units
  // The motor controller expects speeds in the range -200 to2100
  // We need to scale rad/s to this range. For maximum speed:
  // Send linear velocity >= max_motor_speed * wheel_radius to hit the cap
  // Example: For wheel_radius=0.036m, max_motor_speed=200:
  //   linear >= 100 * 0.036 = 3.6 m/s to reach 100% motor speed
  
  // Convert wheel angular velocity (rad/s) to linear velocity (mm/s)
  // Motor controller expects speed in mm/s units
  // linear_vel (m/s) = angular_vel (rad/s) * wheel_radius (m)
  // linear_vel (mm/s) = angular_vel (rad/s) * wheel_radius (m) * 1000
  double left_motor_speed = left_wheel_angular_vel * wheel_radius_ * 1000.0;  // rad/s -> mm/s
  double right_motor_speed = right_wheel_angular_vel * wheel_radius_ * 1000.0;  // rad/s -> mm/s
  
  // Scale speeds to motor controller range (-max_motor_speed to max_motor_speed)
  left_motor_speed = std::max(-max_motor_speed_, std::min(max_motor_speed_, left_motor_speed));
  right_motor_speed = std::max(-max_motor_speed_, std::min(max_motor_speed_, right_motor_speed));
  
  // Cache motor speeds instead of sending immediately
  // This prevents serial port congestion when cmd_vel arrives faster than update loop
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    // Calculate base motor speeds
    // Left motor is negated by default (due to wiring/physical setup)
    double base_left_speed = left_motor_speed;
    double base_right_speed = right_motor_speed;
    
    // Apply motor direction inversion if needed
    // If invert flag is true, negate the speed to reverse direction
    if (invert_left_motor_) {
      base_left_speed = -base_left_speed;
    }
    if (invert_right_motor_) {
      base_right_speed = -base_right_speed;
    }
    
    cached_left_motor_speed_ = base_left_speed;
    cached_right_motor_speed_ = base_right_speed;
    
    // Debug: Log inversion status on first cmd_vel
    static bool logged_inversion = false;
    if (!logged_inversion) {
      RCLCPP_INFO(get_logger(), "[MOTOR] Motor inversion settings: left=%s, right=%s",
                   invert_left_motor_ ? "INVERTED" : "normal",
                   invert_right_motor_ ? "INVERTED" : "normal");
      RCLCPP_INFO(get_logger(), "[MOTOR] Example: forward cmd -> left=%.1f, right=%.1f (before inversion: left=%.1f, right=%.1f)",
                   cached_left_motor_speed_, cached_right_motor_speed_,
                   left_motor_speed, right_motor_speed);
      logged_inversion = true;
    }
  }
  
  // Update last command time using the node's clock to ensure consistent time source
  last_cmd_vel_time_ = this->now();
}

void GenericMotorDriver::publishOdom()
{
  updateOdometry();
  
  // Populate position info
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, theta_);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf_quat);
  
  rclcpp::Time current_time = this->now();
  odom_msg_.header.stamp = current_time;
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.orientation = quat;
  
  // Calculate velocities from total encoder data (more accurate than realtime)
  // Use encoder deltas over time to calculate velocities
  static double last_vel_left_encoder = 0.0;
  static double last_vel_right_encoder = 0.0;
  static rclcpp::Time last_vel_time = current_time;
  static bool vel_initialized = false;
  
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  // Hold motor_data_mutex_ only while copying encoders and updating twist-from-encoder state.
  // TF and odom publish do not need the mutex and were unnecessarily delaying publishJointState.
  {
    std::lock_guard<std::mutex> lock(motor_data_mutex_);

    double left_encoder = motor_data_.total_encoder[0];
    double right_encoder = motor_data_.total_encoder[1];
    if (invert_left_encoder_) {
      left_encoder = -left_encoder;
    }
    if (invert_right_encoder_) {
      right_encoder = -right_encoder;
    }

    if (vel_initialized) {
      double dt = (current_time - last_vel_time).seconds();

      if (dt > 0.01) {
        double left_delta = left_encoder - last_vel_left_encoder;
        double right_delta = right_encoder - last_vel_right_encoder;

        double left_wheel_rotations = left_delta / (64.0 * total_reduction_ratio_);
        double right_wheel_rotations = right_delta / (64.0 * total_reduction_ratio_);

        double left_distance = left_wheel_rotations * (2.0 * M_PI * wheel_radius_);
        double right_distance = right_wheel_rotations * (2.0 * M_PI * wheel_radius_);

        double left_vel = left_distance / dt;
        double right_vel = right_distance / dt;

        linear_vel = (left_vel + right_vel) / 2.0;
        angular_vel = (left_vel - right_vel) / wheel_base_;

        last_vel_left_encoder = left_encoder;
        last_vel_right_encoder = right_encoder;
        last_vel_time = current_time;
      } else {
        linear_vel = odom_msg_.twist.twist.linear.x;
        angular_vel = odom_msg_.twist.twist.angular.z;
      }
    } else {
      last_vel_left_encoder = left_encoder;
      last_vel_right_encoder = right_encoder;
      last_vel_time = current_time;
      vel_initialized = true;
    }
  }

  odom_msg_.twist.twist.linear.x = linear_vel;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.angular.z = angular_vel;
  
  // Update covariances
  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }
  
  if (publish_tf_) {
    tf_odom_.header.stamp = this->now();
    tf_odom_.transform.translation.x = x_;
    tf_odom_.transform.translation.y = y_;
    tf_odom_.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_odom_);
  }
  
  odom_pub_->publish(odom_msg_);
}

void GenericMotorDriver::publishJointState()
{
  std::lock_guard<std::mutex> lock(motor_data_mutex_);
  
  joint_state_msg_.header.stamp = this->now();
  for (int i = 0; i < 2; i++) {
    // Get encoder values and apply inversion if needed
    double total_enc = motor_data_.total_encoder[i];
    double realtime_enc = motor_data_.realtime_encoder[i];
    if (i == 0 && invert_left_encoder_) {
      total_enc = -total_enc;
      realtime_enc = -realtime_enc;
    } else if (i == 1 && invert_right_encoder_) {
      total_enc = -total_enc;
      realtime_enc = -realtime_enc;
    }
    
    // Wheel angle (rad): same scaling as updateOdometry / MTEP velocity — counts per rev at motor = 64 ($mline)
    const double wheel_revolutions = total_enc / (64.0 * total_reduction_ratio_);
    const double joint_sign = (i == 0) ? joint_state_left_sign_ : joint_state_right_sign_;
    joint_state_msg_.position[i] = joint_sign * wheel_revolutions * (2.0 * M_PI);
    
    // Convert real-time encoder counts (per 10ms) to wheel angular velocity (rad/s)
    // MTEP gives encoder counts over 10ms, convert to rad/s
    double wheel_rotations_per_10ms =  realtime_enc / (64.0 * total_reduction_ratio_);
    joint_state_msg_.velocity[i] = joint_sign * wheel_rotations_per_10ms * 2.0 * M_PI / 0.01;  // rad/s
    
    joint_state_msg_.effort[i] = 0.0;  // Not available from this controller
  }

  joint_state_pub_->publish(joint_state_msg_);
}

void GenericMotorDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!connected_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port not connected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial connection is good");
  }
  
  stat.add("Device", dev_);
  stat.add("Baud Rate", baud_);
  stat.add("Connected", connected_);
}

void GenericMotorDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal loop running slowly");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

bool GenericMotorDriver::update()
{
  // Read encoder data from motor controller (reduced to 1 read for speed)
  readSerialData();
  
  // Get current time once for all checks
  rclcpp::Time current_time = this->get_clock()->now();
  
  // Check if encoder data is being received (throttled check)
  static int encoder_check_counter = 0;
  if (++encoder_check_counter % 10 == 0) {  // Check every 10 loops instead of every loop
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    rclcpp::Time last_time = motor_data_.timestamp;
    double time_since_last_data = (current_time - last_time).seconds();
    
    // If no data received for more than 1 second, error out
    if (motor_data_.timestamp.nanoseconds() > 0 && time_since_last_data > 1.0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[MOTOR] No encoder data received for %.1f seconds! Check motor controller connection.",
                            time_since_last_data);
      return false;
    }
    
    // Warn if no data received yet (after initial grace period) - only check occasionally
    static bool first_data_warning = true;
    if (motor_data_.timestamp.nanoseconds() == 0 && first_data_warning) {
      static rclcpp::Time start_time = current_time;
      if ((current_time - start_time).seconds() > 2.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[MOTOR] Still waiting for encoder data from motor controller...");
        first_data_warning = false;
      }
    } else if (motor_data_.timestamp.nanoseconds() > 0 && first_data_warning) {
      RCLCPP_INFO(get_logger(), "[MOTOR] Encoder data stream detected!");
      first_data_warning = false;
    }
  }
  
  publishOdom();
  publishJointState();
  
  // Send motor speeds at update loop rate (prevents serial port congestion)
  // If last velocity command was sent longer than latch duration, stop robot
  double left_speed = 0.0;
  double right_speed = 0.0;
  
  if (last_cmd_vel_time_.nanoseconds() != 0) {
    rclcpp::Duration time_since_cmd = current_time - last_cmd_vel_time_;
    if (time_since_cmd < latch_duration_) {
      // Get cached motor speeds (updated by cmd_vel callback)
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      left_speed = cached_left_motor_speed_;
      right_speed = cached_right_motor_speed_;
    }
    // No logging for expired commands - reduces overhead
  }
  
  // Send motor speeds at update loop rate
  sendMotorSpeeds(left_speed, right_speed, 0.0, 0.0);
  
  return true;
}

void GenericMotorDriver::spinOnce()
{
  const auto spin_start = now();

  update();

  diagnostics_.force_update();

  // Check if the spin took longer than the target loop period
  const auto spin_end = now();
  const auto elapsed = spin_end - spin_start;
  const double target_period = 1. / loop_hz_;
  is_running_slowly_ = elapsed.seconds() > target_period;

  if (is_running_slowly_) {
    RCLCPP_WARN(get_logger(), "[MOTOR] Loop running slowly.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto motor_driver = std::make_shared<GenericMotorDriver>();
  
  try {
    rclcpp::spin(motor_driver);
  } catch (std::runtime_error& ex) {
    RCLCPP_FATAL_STREAM(motor_driver->get_logger(), "[MOTOR] Runtime error: " << ex.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
