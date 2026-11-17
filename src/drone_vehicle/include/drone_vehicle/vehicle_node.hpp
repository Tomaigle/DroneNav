#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "drone_interfaces/msg/error_code.hpp"
#include "drone_interfaces/msg/vehicle_state.hpp"
#include "drone_interfaces/srv/arm.hpp"
#include "drone_interfaces/srv/disarm.hpp"
#include "drone_interfaces/srv/set_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace drone_vehicle {

class VehicleNode : public rclcpp::Node {
public:
  explicit VehicleNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using VehicleState = drone_interfaces::msg::VehicleState;
  using ErrorCode = drone_interfaces::msg::ErrorCode;
  using SetModeSrv = drone_interfaces::srv::SetMode;
  using ArmSrv = drone_interfaces::srv::Arm;
  using DisarmSrv = drone_interfaces::srv::Disarm;
  using MavSetMode = mavros_msgs::srv::SetMode;
  using MavCommandBool = mavros_msgs::srv::CommandBool;

  void state_cb(mavros_msgs::msg::State::ConstSharedPtr msg);
  void pose_cb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void battery_cb(sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  void timer_cb();

  void set_mode_cb(const std::shared_ptr<SetModeSrv::Request> req,
                   std::shared_ptr<SetModeSrv::Response> resp);
  void arm_cb(const std::shared_ptr<ArmSrv::Request> req,
              std::shared_ptr<ArmSrv::Response> resp);
  void disarm_cb(const std::shared_ptr<DisarmSrv::Request> req,
                 std::shared_ptr<DisarmSrv::Response> resp);

  void arm_disarm(bool arm, ErrorCode &error);
  void set_mode(const std::string &mode, ErrorCode &error);

  bool link_ok();
  std::string current_mode();
  bool is_armed();

  rclcpp::CallbackGroup::SharedPtr data_group_;
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Publisher<VehicleState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<SetModeSrv>::SharedPtr set_mode_srv_;
  rclcpp::Service<ArmSrv>::SharedPtr arm_srv_;
  rclcpp::Service<DisarmSrv>::SharedPtr disarm_srv_;

  rclcpp::Client<MavSetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<MavCommandBool>::SharedPtr arming_client_;

  std::mutex data_mutex_;
  mavros_msgs::msg::State::ConstSharedPtr last_state_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_pose_;
  sensor_msgs::msg::BatteryState::ConstSharedPtr last_battery_;
  rclcpp::Time last_state_time_;
  rclcpp::Time last_pose_time_;

  double publish_rate_hz_;
  double pose_timeout_s_;
  double state_timeout_s_;
  double command_timeout_s_;
  int battery_cells_;
};

} // namespace drone_vehicle
