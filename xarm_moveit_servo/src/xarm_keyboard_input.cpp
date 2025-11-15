/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include "xarm_moveit_servo/xarm_keyboard_input.h"
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44
#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_P 0x70
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_0 0x30

#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_MINUS 0x2D
#define KEYCODE_EQUAL 0x3D
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_G 0x67
#define KEYCODE_F 0x66
#define KEYCODE_H 0x68
#define KEYCODE_B 0x62
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D
#define KEYCODE_COMMA 0x2C
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SLASH 0x2F
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

KeyboardReader keyboard_reader_;

KeyboardServoPub::KeyboardServoPub(rclcpp::Node::SharedPtr& node)
: dof_(6), ros_queue_size_(10),
  // make these RELATIVE so they resolve under your node's namespace (e.g., /left_arm/...)
  cartesian_command_in_topic_("cmd_twist/keyboard"),
  joint_command_in_topic_("joint_delta"),
  // leave frames as-is; your launch/YAML can override them
  robot_link_command_frame_("link_base"),
  ee_frame_name_("link_eef"),
  planning_frame_("link_base"),
  joint_vel_cmd_(1.0),
  linear_pos_cmd_(0.5)
{
  node_ = node;
  // before reading params
  joint_prefix_ = "left_arm_";  // default; override per-namespace in launch

  // parameters
  _declare_or_get_param<std::string>(left_arm_ns_, "left_arm_ns", "left_arm");
  _declare_or_get_param<std::string>(right_arm_ns_, "right_arm_ns", "right_arm");
  _declare_or_get_param<std::string>(left_arm_planning_frame_, "left_arm_planning_frame", "openarm_left_link0");
  _declare_or_get_param<std::string>(right_arm_planning_frame_, "right_arm_planning_frame", "openarm_right_link0");
  _declare_or_get_param<std::string>(joint_prefix_, "joint_prefix", joint_prefix_);
  _declare_or_get_param<int>(dof_, "dof", dof_);
  _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
  _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
  _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
  _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
  _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
  _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);
  _declare_or_get_param<std::string>(elevator_cmd_vel_topic_, "elevator_cmd_vel_topic", "/elevator/cmd_vel");
  _declare_or_get_param<double>(elevator_vel_step_, "elevator_vel_step", 0.10);

  // DRIVETRAIN topics & settings
  _declare_or_get_param<std::string>(drivetrain_cmd_vel_topic_, "drivetrain_cmd_vel_topic",
                                     "/drivetrain/cmd_vel");
  _declare_or_get_param<double>(drivetrain_linear_vel_,  "drivetrain_linear_vel",  1.25);
  _declare_or_get_param<double>(drivetrain_angular_vel_, "drivetrain_angular_vel", 0.5);

  // NEW: streaming + watchdog params
  _declare_or_get_param<double>(drivetrain_stream_rate_hz_, "drivetrain_stream_rate_hz", 50.0);  // 50 Hz stream
  _declare_or_get_param<int>(drivetrain_key_hold_ms_, "drivetrain_key_hold_ms", 200);            // 200 ms pulse

  _declare_or_get_param<std::string>(pose_command_in_topic_, "moveit_servo.pose_command_in_topic", "cmd_pose");
  _declare_or_get_param<double>(pose_delta_step_, "pose_delta_step", 0.02);  // Default 2cm step size
  
  // Initialize left arm pose state (simple variables)
  left_arm_pose_initialized_ = false;
  left_arm_x_ = 0.3;
  left_arm_y_ = 0.2;
  left_arm_z_ = 0.3;

  // after creating the other publishers
  const auto left_arm_pose_topic = "/" + left_arm_ns_ + "/" + pose_command_in_topic_;
  pose_pub_left_arm_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(left_arm_pose_topic, 10);

  const auto right_arm_pose_topic = "/" + right_arm_ns_ + "/" + pose_command_in_topic_;
  pose_pub_right_arm_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(right_arm_pose_topic, 10);

  // Publishers for bypassing the bridge (direct to smoothed topic)
  const auto left_arm_pose_smoothed_topic = "/" + left_arm_ns_ + "/" + pose_command_in_topic_ + "/smoothed";
  pose_pub_left_arm_smoothed_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(left_arm_pose_smoothed_topic, 10);

  const auto right_arm_pose_smoothed_topic = "/" + right_arm_ns_ + "/" + pose_command_in_topic_ + "/smoothed";
  pose_pub_right_arm_smoothed_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(right_arm_pose_smoothed_topic, 10);

  // optional: confirm in logs
  RCLCPP_INFO(node_->get_logger(), "Pose pub (left_arm): %s", left_arm_pose_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "Pose pub (right_arm): %s", right_arm_pose_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "Pose pub smoothed (left_arm): %s", left_arm_pose_smoothed_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "Pose pub smoothed (right_arm): %s", right_arm_pose_smoothed_topic.c_str());

  if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
    cartesian_command_in_topic_ = cartesian_command_in_topic_.substr(2);
  }
  if (joint_command_in_topic_.rfind("~/", 0) == 0) {
    joint_command_in_topic_ = joint_command_in_topic_.substr(2);
  }


  // TODO Fix this to use the correct namespaces for the arms
  // Setup pub/sub
  const auto left_arm_twist_topic = "/" + left_arm_ns_ + "/" + cartesian_command_in_topic_;
  const auto right_arm_twist_topic = "/" + right_arm_ns_ + "/" + cartesian_command_in_topic_;
  twist_pub_left_arm_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(left_arm_twist_topic, rclcpp::SensorDataQoS());
  twist_pub_right_arm_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(right_arm_twist_topic, rclcpp::SensorDataQoS());
  const auto left_arm_joint_topic = "/" + left_arm_ns_ + "/" + joint_command_in_topic_;
  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(left_arm_joint_topic, ros_queue_size_);
  elevator_cmd_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64>(elevator_cmd_vel_topic_, 10);
  drivetrain_cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(drivetrain_cmd_vel_topic_, 10);
  
  // Gripper delta command publishers
  const auto left_arm_gripper_delta_topic = "/" + left_arm_ns_ + "/gripper/delta";
  const auto left_arm_gripper_zero_topic = "/" + left_arm_ns_ + "/gripper/zero";
  const auto right_arm_gripper_delta_topic = "/" + right_arm_ns_ + "/gripper/delta";
  const auto right_arm_gripper_zero_topic = "/" + right_arm_ns_ + "/gripper/zero";
  gripper_delta_pub_left_arm_ = node_->create_publisher<std_msgs::msg::Int8>(left_arm_gripper_delta_topic, 10);
  gripper_zero_pub_left_arm_ = node_->create_publisher<std_msgs::msg::String>(left_arm_gripper_zero_topic, 10);
  gripper_delta_pub_right_arm_ = node_->create_publisher<std_msgs::msg::Int8>(right_arm_gripper_delta_topic, 10);
  gripper_zero_pub_right_arm_ = node_->create_publisher<std_msgs::msg::String>(right_arm_gripper_zero_topic, 10);

  // ---- DRIVETRAIN STREAMING TIMER (always publishes) ----
  last_drive_cmd_.header.frame_id = "base_link";
  last_drive_cmd_.twist.linear.x = 0.0;
  last_drive_cmd_.twist.linear.y = 0.0;
  last_drive_cmd_.twist.linear.z = 0.0;
  last_drive_cmd_.twist.angular.x = 0.0;
  last_drive_cmd_.twist.angular.y = 0.0;
  last_drive_cmd_.twist.angular.z = 0.0;
  drive_cmd_expire_ = std::chrono::steady_clock::now(); // already expired => zeros

  auto period_ms = std::chrono::milliseconds(
      static_cast<int>(std::max(1.0, 1000.0 / std::max(1.0, drivetrain_stream_rate_hz_))));
  drivetrain_timer_ = node_->create_wall_timer(
      period_ms,
      [this]() {
        // If the last “non-zero key” command expired, force zeros
        if (std::chrono::steady_clock::now() > drive_cmd_expire_) {
          last_drive_cmd_.twist.linear.x = 0.0;
          last_drive_cmd_.twist.linear.y = 0.0;
          last_drive_cmd_.twist.angular.z = 0.0;
        }
        last_drive_cmd_.header.stamp = node_->now();
        drivetrain_cmd_vel_pub_->publish(last_drive_cmd_);
      });

  // Create a service client to start the ServoServer (optional)
  servo_srv_ns_ = "servo_node";
  bool try_start_service = false;
  _declare_or_get_param<std::string>(servo_srv_ns_, "servo_srv_ns", servo_srv_ns_);
  _declare_or_get_param<bool>(try_start_service, "try_start_service", try_start_service);

  switch_input_left_arm_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
      "/" + left_arm_ns_ + "/" + servo_srv_ns_ + "/switch_command_type");
  switch_input_right_arm_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
      "/" + right_arm_ns_ + "/" + servo_srv_ns_ + "/switch_command_type");

  if (try_start_service) {
    servo_start_client_ = node_->create_client<std_srvs::srv::Trigger>(servo_srv_ns_ + std::string("/start_servo"));
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      servo_start_client_->async_send_request(req);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Start service not available at %s",
                  (servo_srv_ns_ + "/start_servo").c_str());
    }
  }

  // init switching state
  left_arm_command_type_ = -1;
  right_arm_command_type_ = -1;

  RCLCPP_INFO(node_->get_logger(),
      "Twist pubs: %s, %s | Servo switch: /%s/%s/switch_command_type , /%s/%s/switch_command_type",
      left_arm_twist_topic.c_str(), right_arm_twist_topic.c_str(),
      left_arm_ns_.c_str(), servo_srv_ns_.c_str(),
      right_arm_ns_.c_str(), servo_srv_ns_.c_str());
}

template <typename T>
void KeyboardServoPub::_declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
{
  try {
    if (node_->has_parameter(param_name)) {
      node_->get_parameter<T>(param_name, output_value);
    } else {
      output_value = node_->declare_parameter<T>(param_name, default_value);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error getting parameter '" << param_name << "', check parameter type in YAML file");
    throw e;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found parameter - " << param_name << ": " << output_value);
}

void KeyboardServoPub::spin()
{
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }
}

void KeyboardServoPub::publish_pose_left_arm(double x, double y, double z,
                                             double qx, double qy, double qz, double qw)
{
  // 2 = POSE
  _switch_command_type(1, 2);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  // Send pose in the same planning frame you already use
  msg.header.frame_id = left_arm_planning_frame_;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  pose_pub_left_arm_->publish(msg);
}

void KeyboardServoPub::publish_pose_right_arm(double x, double y, double z,
                                             double qx, double qy, double qz, double qw)
{
  // 2 = POSE
  _switch_command_type(2, 2);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  // Send pose in the same planning frame you already use
  msg.header.frame_id = right_arm_planning_frame_;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  pose_pub_right_arm_->publish(msg);
}

void KeyboardServoPub::publish_pose_left_arm_smoothed(double x, double y, double z,
                                                      double qx, double qy, double qz, double qw)
{
  // 2 = POSE
  _switch_command_type(1, 2);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  // Send pose in the same planning frame you already use
  msg.header.frame_id = left_arm_planning_frame_;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  pose_pub_left_arm_smoothed_->publish(msg);
}

void KeyboardServoPub::publish_pose_right_arm_smoothed(double x, double y, double z,
                                                        double qx, double qy, double qz, double qw)
{
  // 2 = POSE
  _switch_command_type(2, 2);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  // Send pose in the same planning frame you already use
  msg.header.frame_id = right_arm_planning_frame_;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  pose_pub_right_arm_smoothed_->publish(msg);
}

void KeyboardServoPub::publish_incremental_left_arm_pose(double dx, double dy, double dz)
{
  if (!left_arm_pose_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Left arm pose not initialized! Press 'C' first.");
    return;
  }
  
  // Update variables
  left_arm_x_ += dx;
  left_arm_y_ += dy;
  left_arm_z_ += dz;
  
  // Publish directly (goes through arm bridge)
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = left_arm_planning_frame_;
  msg.pose.position.x = left_arm_x_;
  msg.pose.position.y = left_arm_y_;
  msg.pose.position.z = left_arm_z_;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  pose_pub_left_arm_->publish(msg);
}

void KeyboardServoPub::_switch_command_type(int arm_idx, int command_type)
{
  int& last_type = (arm_idx == 1) ? left_arm_command_type_ : right_arm_command_type_;
  auto& client   = (arm_idx == 1) ? switch_input_left_arm_  : switch_input_right_arm_;
  const char* arm_label = (arm_idx == 1) ? "left_arm" : "right_arm";

  if (command_type == last_type) return;

  if (!client || !client->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] switch_command_type service unavailable: /%s/%s/switch_command_type",
                arm_label,
                (arm_idx == 1 ? left_arm_ns_.c_str() : right_arm_ns_.c_str()),
                servo_srv_ns_.c_str());
    return;
  }

  auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  switch (command_type) {
    case 0: req->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG; break;
    case 1: req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;     break;
    case 2: req->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;      break;
    default: return;
  }

  auto future = client->async_send_request(req);
  try {
    auto resp = future.get();
    if (resp && resp->success) {
      last_type = command_type;
      RCLCPP_INFO(node_->get_logger(), "[%s] Switched input to %s",
                  arm_label,
                  command_type == 0 ? "JOINT_JOG" :
                  command_type == 1 ? "TWIST"     : "POSE");
    } else {
      RCLCPP_WARN(node_->get_logger(), "[%s] switch_command_type call returned !success", arm_label);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "[%s] switch_command_type exception: %s", arm_label, e.what());
  }
}

void KeyboardServoPub::publish_elevator_velocity(double vz)
{
  if (!elevator_cmd_vel_pub_) return;
  std_msgs::msg::Float64 msg;
  msg.data = vz;
  elevator_cmd_vel_pub_->publish(msg);
}

// UPDATED: don’t publish immediately;
// update "last" command and extend the watchdog expiry.
// The timer will stream it; otherwise it streams zeros.
void KeyboardServoPub::publish_drivetrain_velocity(double linear_x, double linear_y, double angular_z)
{
  if (!drivetrain_cmd_vel_pub_) return;

  last_drive_cmd_.twist.linear.x = linear_x;
  last_drive_cmd_.twist.linear.y = linear_y;
  last_drive_cmd_.twist.linear.z = 0.0;
  last_drive_cmd_.twist.angular.x = 0.0;
  last_drive_cmd_.twist.angular.y = 0.0;
  last_drive_cmd_.twist.angular.z = angular_z;

  // Extend pulse
  drive_cmd_expire_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(drivetrain_key_hold_ms_);
}

// NEW: publish one TwistStamped for a chosen arm (translation only)
void KeyboardServoPub::publish_twist_for_arm(int arm_idx, double dx, double dy, double dz)
{
  _switch_command_type(arm_idx, 1); // 1 = TWIST

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = (arm_idx == 1) ? left_arm_planning_frame_ : right_arm_planning_frame_;
  msg.twist.linear.x = dx;
  msg.twist.linear.y = dy;
  msg.twist.linear.z = dz;

  if (arm_idx == 1) {
    twist_pub_left_arm_->publish(msg);
  } else {
    twist_pub_right_arm_->publish(msg);
  }
}

// Gripper delta command methods
void KeyboardServoPub::publish_gripper_delta(int arm_idx, int8_t delta)
{
  std_msgs::msg::Int8 msg;
  msg.data = delta;
  
  if (arm_idx == 1) {
    gripper_delta_pub_left_arm_->publish(msg);
  } else {
    gripper_delta_pub_right_arm_->publish(msg);
  }
}

void KeyboardServoPub::publish_gripper_zero(int arm_idx)
{
  std_msgs::msg::String msg;
  msg.data = "zero";
  
  if (arm_idx == 1) {
    gripper_zero_pub_left_arm_->publish(msg);
  } else {
    gripper_zero_pub_right_arm_->publish(msg);
  }
}

void KeyboardServoPub::keyLoop()
{
  char c;
  bool publish_joint = false;

  std::thread{ std::bind(&KeyboardServoPub::spin, this) }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Left arm incremental pose control:");
  puts("  C = Initialize/capture starting pose");
  puts("  W/A/S/D = Move in X/Y axes (forward/back, left/right)");
  puts("  Q/E = Move in Z axis (up/down)");
  puts("right_arm (IJKL = X/Y, U/O = Z)");
  puts("Joint jog: 1..6 (prefix from joint_prefix), 'R' flips direction");
  puts("Arrow Up/Down = Elevator velocity (+/-)");
  puts("Drivetrain: T/G = Forward/Back, F/H = Left/Right, B/N = Rotate Left/Right");
  puts("Gripper left_arm: M/, = Open/Close, Z = Zero");
  puts("Gripper right_arm: .// = Open/Close, X = Zero");

  switch_request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

  for (;;) {
    try {
      keyboard_reader_.readOne(&c);
    } catch (const std::runtime_error&) {
      perror("read():");
      return;
    }
    RCLCPP_DEBUG(node_->get_logger(), "value: 0x%02X", c);

    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    {
      case KEYCODE_P:
        publish_pose_left_arm(-0.200, -0.200, 0.600, 0,0,0,1);
        break;
      
      // Initialize left arm pose - set variables and publish
      case KEYCODE_C:
        left_arm_x_ = 0.3;
        left_arm_y_ = 0.2;
        left_arm_z_ = 0.3;
        left_arm_pose_initialized_ = true;
        {
          geometry_msgs::msg::PoseStamped msg;
          msg.header.stamp = node_->now();
          msg.header.frame_id = left_arm_planning_frame_;
          msg.pose.position.x = left_arm_x_;
          msg.pose.position.y = left_arm_y_;
          msg.pose.position.z = left_arm_z_;
          msg.pose.orientation.x = 0.0;
          msg.pose.orientation.y = 0.0;
          msg.pose.orientation.z = 0.0;
          msg.pose.orientation.w = 1.0;
          pose_pub_left_arm_->publish(msg);
        }
        break;
      
      // Left arm incremental pose control (WASDQE)
      case KEYCODE_W:  publish_incremental_left_arm_pose(+pose_delta_step_, 0.0, 0.0); break;  // Forward (X+)
      case KEYCODE_S:  publish_incremental_left_arm_pose(-pose_delta_step_, 0.0, 0.0); break;  // Backward (X-)
      case KEYCODE_A:  publish_incremental_left_arm_pose(0.0, +pose_delta_step_, 0.0); break;  // Left (Y+)
      case KEYCODE_D:  publish_incremental_left_arm_pose(0.0, -pose_delta_step_, 0.0); break;  // Right (Y-)
      case KEYCODE_Q:  publish_incremental_left_arm_pose(0.0, 0.0, +pose_delta_step_); break;  // Up (Z+)
      case KEYCODE_E:  publish_incremental_left_arm_pose(0.0, 0.0, -pose_delta_step_); break;  // Down (Z-)
      // case KEYCODE_W:
      //   publish_pose_right_arm(0.220, -0.325, 0.123, 0.0, 0.0, 0.0, 1.0);
      //   break;
      // case KEYCODE_S:
      //   publish_pose_left_arm(-0.3, -1.2, 0.2, 1.0, 0.0, 0.0, 0.0);
      //   break;
      // case KEYCODE_A:
      //   publish_pose_left_arm(-0.3, -1.0, -0.1, 1.0, 0.0, 0.0, 0.0);
      //   break;
      // case KEYCODE_D:
      //   publish_pose_left_arm(-0.3, -0.8, -0.2, 1.0, 0.0, 0.0, 0.0);
      //   break;
      // case KEYCODE_Q:
      //   publish_pose_left_arm(-0.3, -0.6, 0.0, 0.0, 1.0, 0.0, 0.0);
      //   break;
      // case KEYCODE_E:
      //   publish_pose_left_arm(-0.3, -0.6, 0.0, 0.0, 0.0, 1.0, 0.0);
      //   break;

      // Arm 1 twist
      // case KEYCODE_W:  publish_pose_left_arm_smoothed(0.300, 0.2000, 0.3000, 0.000, 0.707, 0.000, 0.707); break;
      // case KEYCODE_S:  publish_pose_left_arm_smoothed(0.300, 0.2000, 0.3000, 0.000, 0.000, 0.000, 1.000); break;
      // case KEYCODE_A:  publish_pose_left_arm_smoothed(0.300, 0.2000, 0.3000, 0.000, 0.000, 0.707, 0.707); break;
      // case KEYCODE_D:  publish_pose_left_arm_smoothed(0.300, 0.1000, 0.1000, -0.500, 0.500, -0.500, 0.500); break;

      // Right arm pose commands (commented out - now using WASDQE for left arm incremental)
      // case KEYCODE_W:  publish_pose_right_arm_smoothed(0.300, -0.2000, 0.3000, 0.000, 0.707, 0.000, 0.707); break;
      // case KEYCODE_S:  publish_pose_right_arm_smoothed(0.300, -0.2000, 0.3000, 0.000, 0.000, 0.000, 1.000); break;
      // case KEYCODE_A:  publish_pose_right_arm_smoothed(0.300, -0.2000, 0.3000, 0.000, 0.000, 0.707, 0.707); break;
      // case KEYCODE_D:  publish_pose_right_arm_smoothed(0.300, 0.1000, 0.1000, -0.500, 0.500, -0.500, 0.500); break;
      // case KEYCODE_Q:  publish_twist_for_arm(1,  0.0,              0.0,             +linear_pos_cmd_); break;
      // case KEYCODE_E:  publish_twist_for_arm(1,  0.0,              0.0,             -linear_pos_cmd_); break;

      // // Arm 2 twist
      // case KEYCODE_I:  publish_twist_for_arm(2, +linear_pos_cmd_,  0.0,              0.0); break;
      // case KEYCODE_K:  publish_twist_for_arm(2, -linear_pos_cmd_,  0.0,              0.0); break;
      // case KEYCODE_J:  publish_twist_for_arm(2,  0.0,             +linear_pos_cmd_,  0.0); break;
      // case KEYCODE_L:  publish_twist_for_arm(2,  0.0,             -linear_pos_cmd_,  0.0); break;
      // case KEYCODE_U:  publish_twist_for_arm(2,  0.0,              0.0,             +linear_pos_cmd_); break;
      // case KEYCODE_O:  publish_twist_for_arm(2,  0.0,              0.0,             -linear_pos_cmd_); break;

      case KEYCODE_I:  publish_pose_left_arm(-0.200, -0.300, 0.500, 0.500, -0.500, -0.500, 0.500); break;
      case KEYCODE_K:  publish_pose_left_arm(-0.200, -0.300, 0.500, 0.707, 0.000, -0.707, 0.000); break;
      case KEYCODE_J:  publish_pose_left_arm(-0.200, -0.300, 0.500, 0.000, 0.000, -0.707, 0.707); break;
      case KEYCODE_L:  publish_pose_right_arm(0.200, -0.300, 0.500, -0.500, -0.500, -0.500, -0.500); break;
      case KEYCODE_U:  publish_pose_right_arm(0.400, -0.300, 0.500, 0.000, 0.707, 0.000, 0.707); break;
      case KEYCODE_O:  publish_pose_right_arm(0.200, -0.300, 0.500, 0.000, 0.000, -0.707, -0.707); break;

      // Elevator
      case KEYCODE_UP:    publish_elevator_velocity(+elevator_vel_step_); break;
      case KEYCODE_DOWN:  publish_elevator_velocity(-elevator_vel_step_); break;

      // DRIVETRAIN (update last cmd; timer will stream)
      case KEYCODE_T: publish_drivetrain_velocity(+drivetrain_linear_vel_, 0.0, 0.0); break;
      case KEYCODE_G: publish_drivetrain_velocity(-drivetrain_linear_vel_, 0.0, 0.0); break;
      case KEYCODE_F: publish_drivetrain_velocity(0.0, +drivetrain_linear_vel_, 0.0); break;
      case KEYCODE_H: publish_drivetrain_velocity(0.0, -drivetrain_linear_vel_, 0.0); break;
      case KEYCODE_B: publish_drivetrain_velocity(0.0, 0.0, +drivetrain_angular_vel_); break;
      case KEYCODE_N: publish_drivetrain_velocity(0.0, 0.0, -drivetrain_angular_vel_); break;

      // GRIPPER left_arm (M/, = Open/Close, Z = Zero)
      case KEYCODE_M: publish_gripper_delta(1, +1); break;  // Open left_arm gripper
      case KEYCODE_COMMA: publish_gripper_delta(1, -1); break;  // Close left_arm gripper
      case KEYCODE_Z: publish_gripper_zero(1); break;  // Zero left_arm gripper

      // GRIPPER right_arm (.// = Open/Close, X = Zero)
      case KEYCODE_PERIOD: publish_gripper_delta(2, +1); break;  // Open right_arm gripper
      case KEYCODE_SLASH: publish_gripper_delta(2, -1); break;  // Close right_arm gripper
      case KEYCODE_X: publish_gripper_zero(2); break;  // Zero right_arm gripper

      // Joint jog
      case KEYCODE_1: joint_msg->joint_names.push_back("openarm_left_joint1"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_2: joint_msg->joint_names.push_back("openarm_left_joint2"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_3: joint_msg->joint_names.push_back("openarm_left_joint3"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_4: joint_msg->joint_names.push_back("openarm_left_joint4"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_5: joint_msg->joint_names.push_back("openarm_left_joint5"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_6: joint_msg->joint_names.push_back("openarm_left_joint6"); joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_7: joint_msg->joint_names.push_back("joint7");                joint_msg->velocities.push_back(joint_vel_cmd_); publish_joint = true; break;
      case KEYCODE_R: joint_vel_cmd_ *= -1; break;
    }

    if (publish_joint)
    {
      _switch_command_type(1, 0);  // arm 1, JOINT_JOG
      joint_msg->header.stamp = node_->now();
      joint_msg->header.frame_id = "joint";
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }
}

void exit_sig_handler(int sig)
{
  (void)sig;
  keyboard_reader_.shutdown();
  rclcpp::shutdown();
  exit(-1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_moveit_servo_keyboard_node", node_options);

  RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

  KeyboardServoPub keyboard_servo_pub(node);
  signal(SIGINT, exit_sig_handler);
  keyboard_servo_pub.keyLoop();
  keyboard_reader_.shutdown();

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "xarm_moveit_servo_keyboard_node over");
  return 0;
}
