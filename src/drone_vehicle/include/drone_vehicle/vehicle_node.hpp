#pragma once

#include "rclcpp/rclcpp.hpp"

#include "drone_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace drone_vehicle {
class VehicleNode : public rclcpp::Node {
public:
  explicit VehicleNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void state_cb(mavros_msgs::msg::State::ConstSharedPtr msg);
  void pose_cb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void battery_cb(sensor_msgs::msg::BatteryState::ConstSharedPtr msg);

  void timer_cb();

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Publisher<drone_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State::ConstSharedPtr last_state_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_pose_;
  sensor_msgs::msg::BatteryState::ConstSharedPtr last_battery_;
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_state_time_;

  double publish_rate_hz_;
  double pose_timeout_s_;
  double state_timeout_s;
  int battery_cells_;
};
} // namespace drone_vehicle
