#include "drone_vehicle/vehicle_node.hpp"
#include "drone_vehicle/time_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <rcl_action/action_server.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp_action/types.hpp>
#include <stdexcept>
#include <string>

namespace drone_vehicle {

VehicleNode::VehicleNode(const rclcpp::NodeOptions &options)
    : Node("vehicle", options) {
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 10.0);
  pose_timeout_s_ = declare_parameter("pose_timeout_s", 0.5);
  state_timeout_s_ = declare_parameter("state_timeout_s", 3.0);
  command_timeout_s_ = declare_parameter("command_timeout_s", 3.0);
  battery_cells_ = declare_parameter<int>("battery_cells", 6);
  max_alt_m_ = declare_parameter("max_alt_m", 10.0);
  pose_lost_timeout_s_ = declare_parameter("pose_lost_timeout_s", 1.0);
  takeoff_timeout_s_ = declare_parameter("takeoff_timeout_s", 30.0);
  takeoff_alt_tolerance_m_ = declare_parameter("takeoff_alt_tolerance_m", 0.2);

  if (publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be > 0, got: " +
                                std::to_string(publish_rate_hz_));
  }
  if (pose_timeout_s_ <= 0.0) {
    throw std::invalid_argument("pose_timeout_s must be > 0, got: " +
                                std::to_string(pose_timeout_s_));
  }
  if (state_timeout_s_ <= 0.0) {
    throw std::invalid_argument("state_timeout_s must be > 0, got: " +
                                std::to_string(state_timeout_s_));
  }
  if (command_timeout_s_ <= 0.0) {
    throw std::invalid_argument("command_timeout_s must be > 0, got: " +
                                std::to_string(command_timeout_s_));
  }
  if (battery_cells_ <= 0) {
    throw std::invalid_argument("battery_cells must be > 0, got: " +
                                std::to_string(battery_cells_));
  }

  if (max_alt_m_ <= 0.0) {
    throw std::invalid_argument("max_alt_m must be > 0, got: " +
                                std::to_string(max_alt_m_));
  }

  if (takeoff_alt_tolerance_m_ >= max_alt_m_) {
    throw std::invalid_argument("takeoff_alt_tolerance_m_ must be < max_alt_m");
  }

  if (pose_lost_timeout_s_ <= 0.0) {
    throw std::invalid_argument("pose_lost_timeout_s_ must be > 0, got: " +
                                std::to_string(pose_lost_timeout_s_));
  }

  if (takeoff_timeout_s_ <= 0.0) {
    throw std::invalid_argument("takeoff_timeout_s must be > 0, got: " +
                                std::to_string(takeoff_alt_tolerance_m_));
  }

  if (takeoff_alt_tolerance_m_ <= 0.0) {
    throw std::invalid_argument("takeoff_alt_tolerance_m_ must be > 0, got: " +
                                std::to_string(takeoff_alt_tolerance_m_));
  }

  for (const auto &[name, value] :
       get_node_parameters_interface()->get_parameter_overrides()) {
    if (!has_parameter(name)) {
      RCLCPP_WARN(get_logger(), "unknown parameter '%s' ignored (typo?)",
                  name.c_str());
    }
  }

  data_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  server_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  arming_client_ = create_client<MavCommandBool>(
      "/mavros/cmd/arming", rclcpp::ServicesQoS(), client_group_);

  takeoff_client_ = create_client<MavCommandTOL>(
      "/mavros/cmd/takeoff", rclcpp::ServicesQoS(), client_group_);

  set_mode_srv_ = create_service<SetModeSrv>(
      "/vehicle/set_mode",
      [this](const std::shared_ptr<SetModeSrv::Request> req,
             std::shared_ptr<SetModeSrv::Response> resp) {
        set_mode_cb(req, resp);
      },
      rclcpp::ServicesQoS(), server_group_);

  arm_srv_ = create_service<ArmSrv>(
      "/vehicle/arm",
      [this](const std::shared_ptr<ArmSrv::Request> req,
             std::shared_ptr<ArmSrv::Response> resp) { arm_cb(req, resp); },
      rclcpp::ServicesQoS(), server_group_);

  disarm_srv_ = create_service<DisarmSrv>(
      "/vehicle/disarm",
      [this](const std::shared_ptr<DisarmSrv::Request> req,
             std::shared_ptr<DisarmSrv::Response> resp) {
        disarm_cb(req, resp);
      },
      rclcpp::ServicesQoS(), server_group_);

  takeoff_server_ = rclcpp_action::create_server<TakeOffAction>(
      this, "/vehicle/takeoff",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const TakeOffAction::Goal> goal) {
        return takeoff_handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
        return takeoff_handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
        return takeoff_handle_accepted(goal_handle);
      },
      rcl_action_server_get_default_options(), action_group_);
  RCLCPP_INFO(get_logger(), "vehicle node started");
}

VehicleNode::~VehicleNode() {
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
}

bool VehicleNode::link_ok() {
  mavros_msgs::msg::State::ConstSharedPtr state;
  SteadyTp stamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state = last_state_;
    stamp = last_state_time_;
  }
  return state and state->connected and age_s(stamp) < state_timeout_s_;
}

std::string VehicleNode::current_mode() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_ ? last_state_->mode : "";
}

bool VehicleNode::is_armed() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_ and last_state_->armed;
}

bool VehicleNode::pose_fresh() { return current_pose().has_value(); }

bool VehicleNode::ready_for_pos_control() { return link_ok() and pose_fresh(); }

bool VehicleNode::fail_if_error(
    const ErrorCode &err, const std::shared_ptr<GoalHandleTakeoff> &goal_handle,
    const std::shared_ptr<TakeOffAction::Result> &result) {

  if ((err.code == ErrorCode::CODE_OK)) {
    return false;
  }
  result->error = err;
  goal_handle->abort(result);
  return true;
}

std::optional<geometry_msgs::msg::Pose> VehicleNode::current_pose() {
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  SteadyTp stamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pose = last_pose_;
    stamp = last_pose_time_;
  }
  if (!pose or (age_s(stamp) >= pose_timeout_s_)) {
    return std::nullopt;
  }
  return pose->pose;
}
void VehicleNode::state_cb(mavros_msgs::msg::State::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "first state received");

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (last_state_ and last_state_->mode != msg->mode) {
    RCLCPP_INFO(get_logger(), "mode: %s -> %s", last_state_->mode.c_str(),
                msg->mode.c_str());
  }
  if (last_state_ and last_state_->connected != msg->connected) {
    RCLCPP_WARN(get_logger(), "FCU %s",
                msg->connected ? "connected" : "disconnected");
  }

  last_state_ = std::move(msg);
  last_state_time_ = std::chrono::steady_clock::now();
}

void VehicleNode::pose_cb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "first pose received");

  std::lock_guard<std::mutex> lock(data_mutex_);
  last_pose_ = std::move(msg);
  last_state_time_ = std::chrono::steady_clock::now();
}

void VehicleNode::battery_cb(
    sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(), "first battery received");

  std::lock_guard<std::mutex> lock(data_mutex_);
  last_battery_ = std::move(msg);
}

// TODO: real readiness/source from /mavros/estimator_status (EKF flags)
void VehicleNode::timer_cb() {
  mavros_msgs::msg::State::ConstSharedPtr state;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  sensor_msgs::msg::BatteryState::ConstSharedPtr battery;
  SteadyTp state_time, pose_time;
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

  if (state) {
    const bool state_fresh = age_s(state_time) < state_timeout_s_;
    if (!state_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "state stale");
    }
    msg.armed = state->armed;
    msg.mode = state->mode;
    msg.link_ok = link_ok();
  } else {
    msg.armed = false;
    msg.mode = "";
    msg.link_ok = false;
  }

  if (battery) {
    msg.battery_percent = battery->percentage;
    msg.battery_voltage = battery->voltage;
    msg.battery_cell_voltage = battery->voltage / battery_cells_;
  } else {
    msg.battery_percent = std::numeric_limits<float>::quiet_NaN();
    msg.battery_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.battery_cell_voltage = std::numeric_limits<float>::quiet_NaN();
  }

  bool pose_f = pose_fresh();
  if (pose) {
    msg.pose = pose->pose;
    if (!pose_f) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pose stale");
    }
  }

  msg.position_source = pose_f ? VehicleState::POSITION_SOURCE_GPS
                               : VehicleState::POSITION_SOURCE_NONE;
  msg.ready_for_position_control = ready_for_pos_control();

  state_pub_->publish(msg);
}

} // namespace drone_vehicle
