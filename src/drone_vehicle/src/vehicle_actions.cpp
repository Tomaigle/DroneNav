#include "drone_vehicle/vehicle_node.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp_action/server.hpp>
#include <thread>

namespace drone_vehicle {
rclcpp_action::GoalResponse VehicleNode::takeoff_handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TakeOffAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received takeoff request to %.2fm",
              goal->alt_relative_m);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VehicleNode::takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> /*goal_handle*/) {
  RCLCPP_INFO(this->get_logger(), "Takeoff: received cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}
void VehicleNode::takeoff_execute(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing takeoff");

  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<TakeOffAction::Feedback>();
  auto &current_alt_m = feedback->current_alt_m;
  current_alt_m = 0.0f;
  auto result = std::make_shared<TakeOffAction::Result>();

  while (rclcpp::ok() and current_alt_m < goal->alt_relative_m) {
    if (goal_handle->is_canceling()) {
      result->final_alt_m = current_alt_m;
      RCLCPP_INFO(this->get_logger(), "Takeoff: canceled at: %.2fm",
                  current_alt_m);
      result->error.code = ErrorCode::CODE_PREEMPTED;
      goal_handle->canceled(result);
      return;
    }
    current_alt_m += 0.1f;
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
  result->error.code = ErrorCode::CODE_OK;
  result->final_alt_m = current_alt_m;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Takeoff: reached %.2fm", current_alt_m);
}

void VehicleNode::takeoff_handle_accepted(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
  std::thread{[this, goal_handle] {
    VehicleNode::takeoff_execute(goal_handle);
  }}.detach();
}
} // namespace drone_vehicle
