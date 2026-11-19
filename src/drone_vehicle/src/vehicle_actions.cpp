#include "drone_vehicle/time_utils.hpp"
#include "drone_vehicle/vehicle_node.hpp"
#include <chrono>
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
  if (goal->alt_relative_m <= 0) {
    RCLCPP_WARN(this->get_logger(), "Target alt must be > 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->alt_relative_m > max_alt_m_) {
    RCLCPP_WARN(this->get_logger(), "Target alt must be <= max alt");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!link_ok()) {
    RCLCPP_WARN(this->get_logger(), "Link lost");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!ready_for_pos_control()) {
    RCLCPP_WARN(this->get_logger(), "Vehicle not ready for positon control");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VehicleNode::takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> /*goal_handle*/) {
  RCLCPP_INFO(this->get_logger(), "Takeoff: received cancel");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VehicleNode::takeoff_execute(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
  RCLCPP_INFO(get_logger(), "Executing takeoff");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<TakeOffAction::Result>();
  auto feedback = std::make_shared<TakeOffAction::Feedback>();
  ErrorCode err;

  const auto start_pose = current_pose();
  if (!start_pose) {
    RCLCPP_WARN(get_logger(), "takeoff: no fresh pose, aborting");
    result->error.code = ErrorCode::CODE_NOT_READY;
    goal_handle->abort(result);
    return;
  }
  const double start_alt = start_pose->position.z;

  set_mode("GUIDED", err);
  if (fail_if_error(err, goal_handle, result)) {
    return;
  }

  if (goal_handle->is_canceling()) {
    result->error.code = ErrorCode::CODE_PREEMPTED;
    goal_handle->canceled(result);
    return;
  }

  arm_disarm(true, err);
  if (fail_if_error(err, goal_handle, result)) {
    return;
  }

  takeoff(goal->alt_relative_m, err);

  auto hold_and_abort = [&](uint8_t code, const char *why) {
    ErrorCode stop_err;
    set_mode("LOITER", stop_err);
    RCLCPP_ERROR(get_logger(), "takeoff: %s", why);
    result->error.code = code;
    result->final_alt_m = feedback->current_alt_m;
    goal_handle->abort(result);
  };

  rclcpp::Rate loop_rate(10);
  auto pose_lost_deadline = deadline_after(pose_lost_timeout_s_);
  const auto takeoff_deadline = deadline_after(takeoff_timeout_s_);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      ErrorCode stop_err;
      set_mode("LOITER", stop_err);
      if (stop_err.code != ErrorCode::CODE_OK) {
        RCLCPP_ERROR(get_logger(),
                     "takeoff cancel: LOITER failed, pilot take over");
      }
      result->error.code = ErrorCode::CODE_PREEMPTED;
      result->final_alt_m = feedback->current_alt_m;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "takeoff: canceled at %.2f m",
                  feedback->current_alt_m);
      return;
    }

    if (std::chrono::steady_clock::now() > takeoff_deadline) {
      return hold_and_abort(ErrorCode::CODE_TIMEOUT, "timeout");
    }

    const auto pose = current_pose();
    if (pose) {
      pose_lost_deadline = deadline_after(pose_lost_timeout_s_);
      feedback->current_alt_m =
          static_cast<float>(pose->position.z - start_alt);
    } else if (std::chrono::steady_clock::now() > pose_lost_deadline) {
      return hold_and_abort(ErrorCode::CODE_NOT_READY,
                            "pose lost while climbing");
    } else {
      loop_rate.sleep();
      continue;
    }

    if (feedback->current_alt_m >=
        goal->alt_relative_m - takeoff_alt_tolerance_m_) {
      break;
    }

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (!rclcpp::ok()) {
    result->error.code = ErrorCode::CODE_PREEMPTED;
    goal_handle->abort(result);
    return;
  }

  result->error.code = ErrorCode::CODE_OK;
  result->final_alt_m = feedback->current_alt_m;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "takeoff: reached %.2f m", feedback->current_alt_m);
}

void VehicleNode::takeoff_handle_accepted(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
  std::thread{[this, goal_handle] {
    VehicleNode::takeoff_execute(goal_handle);
  }}.detach();
}
} // namespace drone_vehicle
