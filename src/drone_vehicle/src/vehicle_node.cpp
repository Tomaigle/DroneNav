#include "drone_vehicle/vehicle_node.hpp"
#include "drone_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <stdexcept>
#include <string>

namespace drone_vehicle {
using drone_interfaces::msg::VehicleState;

VehicleNode::VehicleNode(const rclcpp::NodeOptions &options)
    : Node("vehicle", options) {
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 10.0);
  pose_timeout_s_ = declare_parameter("pose_timeout_s", 0.5);
  state_timeout_s_ = declare_parameter("state_timeout_s", 3.0);
  battery_cells_ = declare_parameter<int>("battery_cells", 6);
  command_timeout_s_ = declare_parameter("command_timeout_s", 3.0);

  for (const auto &[name, value] :
       get_node_parameters_interface()->get_parameter_overrides()) {
    if (!has_parameter(name)) {
      RCLCPP_WARN(this->get_logger(), "Unknown parameter '%s' ignored",
                  name.c_str());
    }
  }

  last_state_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_pose_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  data_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  server_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("Publish rate must be > 0, got: " +
                                std::to_string(publish_rate_hz_));
  }

  if (pose_timeout_s_ <= 0) {
    throw std::invalid_argument("pose timeout must be > 0, got: " +
                                std::to_string(pose_timeout_s_));
  }
  if (battery_cells_ <= 0) {
    throw std::invalid_argument("battery cells must be > 0, got: " +
                                std::to_string(battery_cells_));
  }

  if (state_timeout_s_ <= 0) {
    throw std::invalid_argument("state timeout must be >0, got: " +
                                std::to_string(state_timeout_s_));
  }

  if (command_timeout_s_ <= 0) {
    throw std::invalid_argument("command timeout must be > 0" +
                                std::to_string(state_timeout_s_));
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = data_group_;
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
        state_cb(std::move(msg));
      },
      sub_opts);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        pose_cb(std::move(msg));
      },
      sub_opts);

  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
        battery_cb(std::move(msg));
      },
      sub_opts);

  state_pub_ = create_publisher<VehicleState>("/vehicle/state", 10);

  timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() { timer_cb(); }, data_group_);

  set_mode_client_ = create_client<MavSetMode>(
      "/mavros/set_mode", rclcpp::ServicesQoS(), client_group_);

  set_mode_srv_ = create_service<SetModeSrv>(
      "/vehicle/set_mode",
      [this](const std::shared_ptr<SetModeSrv::Request> req,
             std::shared_ptr<SetModeSrv::Response> resp) {
        set_mode_cb(req, resp);
      },
      rclcpp::ServicesQoS(), server_group_);

  RCLCPP_INFO(this->get_logger(), "vehicle node started");
}

bool VehicleNode::link_ok() {
  mavros_msgs::msg::State::ConstSharedPtr state;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = last_state_;
    stamp = last_state_time_;
  }
  return state and state->connected and
         (now() - stamp).seconds() < state_timeout_s_;
}

std::string VehicleNode::current_mode() {
  mavros_msgs::msg::State::ConstSharedPtr state;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = last_state_;
  }
  return state ? state->mode : "";
}

void VehicleNode::state_cb(mavros_msgs::msg::State::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "First state received");

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (last_state_ and last_state_->mode != msg->mode) {
    RCLCPP_INFO(this->get_logger(), "Mode: %s to %s", last_state_->mode.c_str(),
                msg->mode.c_str());
  }

  if (last_state_ and last_state_->connected != msg->connected) {
    RCLCPP_WARN(this->get_logger(), "FCU %s",
                msg->connected ? "connected" : "disconnected");
  }

  last_state_ = std::move(msg);
  last_state_time_ = now();
}

void VehicleNode::pose_cb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "First pose received");

  std::lock_guard<std::mutex> lock(data_mutex_);

  last_pose_ = std::move(msg);
  last_pose_time_ = now();
}

void VehicleNode::battery_cb(
    sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "First battery info received");

  std::lock_guard<std::mutex> lock(data_mutex_);

  last_battery_ = std::move(msg);
}

/**
 * @brief TODO: add real check from /mavros/estimator_status with ekf flags
 */
void VehicleNode::timer_cb() {

  mavros_msgs::msg::State::ConstSharedPtr state;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  sensor_msgs::msg::BatteryState::ConstSharedPtr battery;
  rclcpp::Time state_time, pose_time;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = last_state_;
    pose = last_pose_;
    battery = last_battery_;
    state_time = last_state_time_;
    pose_time = last_pose_time_;
  }

  VehicleState msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  bool state_fresh = false;

  if (state) {
    msg.armed = state->armed;
    msg.mode = state->mode;
    state_fresh = (now() - state_time).seconds() < state_timeout_s_;

    if (!state_fresh) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                           "state stale");
    }
    msg.link_ok = state->connected and state_fresh;
  } else {
    msg.armed = false;
    msg.mode = "";
    msg.link_ok = false;
  }

  if (battery) {
    msg.battery_percent = battery->percentage;
    msg.battery_voltage = battery->voltage;
    msg.battery_cell_voltage = msg.battery_voltage / battery_cells_;
  } else {
    msg.battery_percent = std::numeric_limits<float>::quiet_NaN();
    msg.battery_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.battery_cell_voltage = std::numeric_limits<float>::quiet_NaN();
  }

  bool pose_fresh = false;
  if (pose) {
    msg.pose = pose->pose;
    pose_fresh = (now() - pose_time).seconds() < pose_timeout_s_;

    if (!pose_fresh) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                           "pose stale");
    }
  }

  msg.position_source = pose_fresh ? VehicleState::POSITION_SOURCE_GPS
                                   : VehicleState::POSITION_SOURCE_NONE;
  msg.ready_for_position_control = msg.link_ok and pose_fresh;

  state_pub_->publish(msg);
}

} // namespace drone_vehicle
