#include "drone_vehicle/vehicle_node.hpp"
#include "drone_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <rclcpp/node_options.hpp>

namespace drone_vehicle {
using drone_interfaces::msg::VehicleState;

VehicleNode::VehicleNode(const rclcpp::NodeOptions &options)
    : Node("vehicle", options) {
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 10.0);
  pose_timeout_s_ = declare_parameter("pose_timeout_s", 0.5);

  state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
        state_cb(std::move(msg));
      });

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        pose_cb(std::move(msg));
      });

  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
        battery_cb(std::move(msg));
      });

  state_pub_ = create_publisher<VehicleState>("/vehicle/state", 10);

  timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                        [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "vehicle node started");
}

} // namespace drone_vehicle
