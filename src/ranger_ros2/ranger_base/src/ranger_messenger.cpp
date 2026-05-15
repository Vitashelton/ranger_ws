/**
 * @file ranger_messenger.cpp
 * @date 2021-04-20
 * @brief ROS2 messenger for AgileX / Weston Robot Ranger bases.
 *
 * Modified for Ranger Mini 2.0 navigation usage:
 * - keeps official ranger_ros2 /cmd_vel -> CAN bridge behavior;
 * - fixes Ranger Mini V2 parameter selection flow;
 * - improves /cmd_vel mapping for Ranger Mini 2.0 Ackermann / oblique / spin modes;
 * - handles pure lateral cmd_vel safely without atan(y / 0);
 * - normalizes BatteryState percentage to 0.0 ~ 1.0;
 * - avoids invalid NaN assignment to uint8_t BatteryState::present;
 * - keeps odometry compatible with Nav2.
 *
 * @copyright Copyright (c) 2021 AgileX Robotics
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#include "ranger_base/ranger_messenger.hpp"
#include "ranger_base/kinematics_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {

namespace {
constexpr double kLinearDeadband = 1e-4;
constexpr double kAngularDeadband = 1e-4;
constexpr double kModeWarnThrottleMs = 2000.0;

inline double Clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

inline double NormalizeBatteryPercentage(double soc) {
  // sensor_msgs/BatteryState.percentage expects 0.0 ~ 1.0.
  // Some Ranger BMS feedback reports SOC as 0 ~ 100.
  if (std::isnan(soc)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (soc > 1.0) {
    return Clamp(soc / 100.0, 0.0, 1.0);
  }
  return Clamp(soc, 0.0, 1.0);
}
}  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node) {
  node_ = node;
  LoadParameters();

  // Connect to robot and setup ROS subscription.
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect to the CAN port: %s",
                   port_name_.c_str());
      return;
    }

    // Ranger Mini powers on in standby / remote-priority behavior.
    // EnableCommandedMode switches the chassis to CAN command mode through ugv_sdk.
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid port name: %s", port_name_.c_str());
    return;
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  // Load parameters from launch files.
  port_name_ = node_->declare_parameter<std::string>("port_name", "can0");

  // Keep compatibility with the README wording in some ranger_ros2 versions.
  const std::string can_device =
      node_->declare_parameter<std::string>("can_device", port_name_);
  if (port_name_ == "can0" && can_device != port_name_) {
    port_name_ = can_device;
  }

  robot_model_ = node_->declare_parameter<std::string>("robot_model", "ranger");
  odom_frame_ = node_->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf", false);

  RCLCPP_INFO(node_->get_logger(),
              "Successfully loaded parameters:\n"
              "  port_name: %s\n"
              "  robot_model: %s\n"
              "  odom_frame: %s\n"
              "  base_frame: %s\n"
              "  update_rate: %d\n"
              "  odom_topic_name: %s\n"
              "  publish_odom_tf: %d",
              port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
              base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
              publish_odom_tf_);

  // Load robot parameters.
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;
    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central = RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel = RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann = RangerMiniV1Params::max_steer_angle_ackermann;
  } else if (robot_model_ == "ranger_mini_v2") {
    robot_type_ = RangerSubType::kRangerMiniV2;
    robot_params_.track = RangerMiniV2Params::track;
    robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
    robot_params_.max_steer_angle_central = RangerMiniV2Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel = RangerMiniV2Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann = RangerMiniV2Params::max_steer_angle_ackermann;
  } else if (robot_model_ == "ranger_mini_v3") {
    robot_type_ = RangerSubType::kRangerMiniV3;
    robot_params_.track = RangerMiniV3Params::track;
    robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
    robot_params_.max_steer_angle_central = RangerMiniV3Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel = RangerMiniV3Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann = RangerMiniV3Params::max_steer_angle_ackermann;
  } else {
    robot_type_ = RangerSubType::kRanger;
    robot_params_.track = RangerParams::track;
    robot_params_.wheelbase = RangerParams::wheelbase;
    robot_params_.max_linear_speed = RangerParams::max_linear_speed;
    robot_params_.max_angular_speed = RangerParams::max_angular_speed;
    robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
    robot_params_.max_steer_angle_central = RangerParams::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel = RangerParams::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerParams::max_round_angle;
    robot_params_.min_turn_radius = RangerParams::min_turn_radius;
    robot_params_.max_steer_angle_ackermann = RangerParams::max_steer_angle_ackermann;
  }

  parking_mode_ = false;
}

void RangerROSMessenger::SetupSubscription() {
  // Publishers.
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>("/system_state", 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

  // Subscriber.
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5,
      std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // Update odometry.
  {
    const double dt = (current_time_ - last_time_).seconds();
    if (dt > 0.0) {
      UpdateOdometry(state.motion_state.linear_velocity,
                     state.motion_state.angular_velocity,
                     state.motion_state.steering_angle, dt);
    }
    last_time_ = current_time_;
  }

  // Publish system state.
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // Publish motion mode.
  {
    motion_mode_ = state.motion_mode_state.motion_mode;

    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // Publish actuator state.
  {
    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;

    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage = actuator_state.actuator_ls_state->driver_voltage;
      driver_state_msg.driver_temperature = actuator_state.actuator_ls_state->driver_temp;
      driver_state_msg.motor_temperature = actuator_state.actuator_ls_state->motor_temp;
      driver_state_msg.driver_state = actuator_state.actuator_ls_state->driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state->current;
      motor_state_msg.pulse_count = actuator_state.actuator_hs_state->pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state->rpm;

      // The original upstream code exposes only aggregate SDK pointers here.
      // Keep this behavior to avoid breaking ABI. For per-wheel values, prefer
      // adding dedicated fields from motor_angles / motor_speeds after checking
      // the exact ugv_sdk structure used in your workspace.
      motor_state_msg.motor_angles = actuator_state.motor_angles.angle_5;
      motor_state_msg.motor_speeds = actuator_state.motor_speeds.speed_1;

      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // Publish BMS state.
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = NormalizeBatteryPercentage(
        common_sensor_state.bms_basic_state.battery_soc);
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = true;

    if (!std::isnan(batt_msg.percentage) && batt_msg.percentage <= 0.15) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Ranger Mini battery SOC is low: %.1f%%",
                           batt_msg.percentage * 100.0);
    }

    battery_state_pub_->publish(batt_msg);
  }
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double angle, double dt) {
  // Update odometry calculations.
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = ConvertInnerAngleToCentral(angle);

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(robot_params_.wheelbase, u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    u.phi = (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) ? M_PI / 2.0 : angle;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }

  // Publish odometry and tf messages.
  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z =
        2.0 * linear * std::sin(ConvertInnerAngleToCentral(angle)) /
        robot_params_.wheelbase;
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    const double phi =
        (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) ? M_PI / 2.0 : angle;

    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);
    odom_msg.twist.twist.angular.z = 0.0;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular;
  }

  odom_pub_->publish(odom_msg);

  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!msg) {
    return;
  }

  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  }

  const double vx = msg->linear.x;
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;
  const double planar_speed = std::hypot(vx, vy);

  const bool has_vx = std::abs(vx) > kLinearDeadband;
  const bool has_vy = std::abs(vy) > kLinearDeadband;
  const bool has_wz = std::abs(wz) > kAngularDeadband;
  const bool has_translation = planar_speed > kLinearDeadband;

  // Full stop.
  if (!has_translation && !has_wz) {
    robot_->SetMotionCommand(0.0, 0.0, 0.0);
    return;
  }

  // Ranger Mini 2.0 CAN motion modes are mode-based: Ackermann, oblique/parallel,
  // or spin. It cannot execute arbitrary holonomic vx + vy + wz at the same time
  // through the standard CAN command frame. For Nav2, prefer translation when vy
  // exists, and ignore wz in this command cycle.
  if (has_vy) {
    motion_mode_ = MotionState::MOTION_MODE_PARALLEL;
    robot_->SetMotionMode(MotionState::MOTION_MODE_PARALLEL);

    if (has_wz) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), kModeWarnThrottleMs,
          "Ranger Mini oblique mode cannot combine lateral translation and yaw; "
          "executing vx/vy and ignoring angular.z for this command.");
    }

    // Convert desired body-frame vector to chassis oblique command:
    // - speed is signed along the wheel direction;
    // - steering angle is kept inside [-90 deg, 90 deg].
    double speed_cmd = planar_speed;
    double steer_cmd = std::atan2(vy, vx);

    if (steer_cmd > M_PI / 2.0) {
      steer_cmd -= M_PI;
      speed_cmd = -speed_cmd;
    } else if (steer_cmd < -M_PI / 2.0) {
      steer_cmd += M_PI;
      speed_cmd = -speed_cmd;
    }

    steer_cmd = Clamp(steer_cmd,
                      -robot_params_.max_steer_angle_parallel,
                      robot_params_.max_steer_angle_parallel);
    speed_cmd = Clamp(speed_cmd,
                      -robot_params_.max_linear_speed,
                      robot_params_.max_linear_speed);

    robot_->SetMotionCommand(speed_cmd, steer_cmd);
    return;
  }

  // Pure rotation or rotation with too-small translational radius: spin mode.
  double steer_cmd = 0.0;
  double radius = std::numeric_limits<double>::infinity();

  if (!has_vx && has_wz) {
    motion_mode_ = MotionState::MOTION_MODE_SPINNING;
    robot_->SetMotionMode(MotionState::MOTION_MODE_SPINNING);

    double angular_cmd = Clamp(wz,
                               -robot_params_.max_angular_speed,
                               robot_params_.max_angular_speed);
    robot_->SetMotionCommand(0.0, 0.0, angular_cmd);
    return;
  }

  steer_cmd = CalculateSteeringAngle(*msg, radius);

  if (has_wz && radius < robot_params_.min_turn_radius) {
    motion_mode_ = MotionState::MOTION_MODE_SPINNING;
    robot_->SetMotionMode(MotionState::MOTION_MODE_SPINNING);

    double angular_cmd = Clamp(wz,
                               -robot_params_.max_angular_speed,
                               robot_params_.max_angular_speed);
    robot_->SetMotionCommand(0.0, 0.0, angular_cmd);
    return;
  }

  // Default forward/backward Ackermann mode.
  motion_mode_ = MotionState::MOTION_MODE_DUAL_ACKERMAN;
  robot_->SetMotionMode(MotionState::MOTION_MODE_DUAL_ACKERMAN);

  steer_cmd = Clamp(steer_cmd,
                    -robot_params_.max_steer_angle_ackermann,
                    robot_params_.max_steer_angle_ackermann);

  double linear_cmd = Clamp(vx,
                            -robot_params_.max_linear_speed,
                            robot_params_.max_linear_speed);

  robot_->SetMotionCommand(linear_cmd, steer_cmd);
}

geometry_msgs::msg::Quaternion RangerROSMessenger::createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                                  double& radius) {
  const double linear = std::abs(msg.linear.x);
  const double angular = std::abs(msg.angular.z);

  if (linear < kLinearDeadband || angular < kAngularDeadband) {
    radius = std::numeric_limits<double>::infinity();
    return 0.0;
  }

  // Circular motion radius from cmd_vel.
  radius = linear / angular;

  // Positive angular.z with positive forward velocity should turn left in ROS.
  // Keep the upstream sign convention to match existing Ranger behavior.
  const int sign = (msg.angular.z * msg.linear.x) >= 0.0 ? 1 : -1;

  const double phi_i = std::atan((robot_params_.wheelbase / 2.0) / radius);

  // CAN protocol limits front/rear Ackermann command steering angle to +/-40 deg.
  const double max_phi_rad = 40.0 * M_PI / 180.0;
  return sign * std::min(phi_i, max_phi_rad);
}

double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0.0;
  const double phi_i = std::abs(angle);

  phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                  (robot_params_.wheelbase * std::cos(phi_i) +
                   robot_params_.track * std::sin(phi_i)));

  phi *= angle >= 0.0 ? 1.0 : -1.0;
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  const double phi = std::abs(angle);

  double phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                           (robot_params_.wheelbase * std::cos(phi) -
                            robot_params_.track * std::sin(phi)));

  phi_i *= angle >= 0.0 ? 1.0 : -1.0;
  return phi_i;
}

}  // namespace westonrobot
