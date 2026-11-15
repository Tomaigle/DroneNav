#pragma once

#include "rclcpp/rclcpp.hpp"

#include "drone_interfaces/msg/vehicle_state.hpp"
#include "drone_interfaces/srv/set_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <mutex>

namespace drone_vehicle {
class VehicleNode : public rclcpp::Node {
public:
  explicit VehicleNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using SetModeSrv = drone_interfaces::srv::SetMode;
  using MavSetMode = mavros_msgs::srv::SetMode;

  void state_cb(mavros_msgs::msg::State::ConstSharedPtr msg);
  void pose_cb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void battery_cb(sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  void set_mode_cb(const std::shared_ptr<SetModeSrv::Request> req,
                   std::shared_ptr<SetModeSrv::Response> resp);

  void timer_cb();

  bool link_ok();
  std::string current_mode();

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Service<SetModeSrv>::SharedPtr set_mode_srv_;
  rclcpp::Client<MavSetMode>::SharedPtr set_mode_client_;

  rclcpp::Publisher<drone_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State::ConstSharedPtr last_state_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_pose_;
  sensor_msgs::msg::BatteryState::ConstSharedPtr last_battery_;
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_state_time_;

  double publish_rate_hz_;
  double pose_timeout_s_;
  double state_timeout_s_;
  double command_timeout_s_;
  int battery_cells_;

  rclcpp::CallbackGroup::SharedPtr data_group_;
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;

  std::mutex data_mutex_;
};
} // namespace drone_vehicle
