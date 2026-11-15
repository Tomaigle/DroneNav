#include "drone_interfaces/msg/error_code.hpp"
#include "drone_vehicle/vehicle_node.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>

namespace drone_vehicle {
using drone_interfaces::msg::ErrorCode;

namespace {
void finish(const rclcpp::Logger &logger, ErrorCode &error, uint8_t code,
            const std::string &service, const std::string &why) {
  error.code = code;
  if (code == ErrorCode::CODE_OK) {
    RCLCPP_INFO(logger, "%s: %s", service.c_str(), why.c_str());
  } else {
    RCLCPP_WARN(logger, "%s: %s", service.c_str(), why.c_str());
  }
}

std::chrono::steady_clock::time_point deadline_after(double seconds) {
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(seconds));
}
} // namespace

void VehicleNode::set_mode_cb(const std::shared_ptr<SetModeSrv::Request> req,
                              std::shared_ptr<SetModeSrv::Response> resp) {
  const std::string srv = "set_mode " + req->mode;

  if (req->mode.empty()) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                  "empty mode");
  }
  if (!link_ok()) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_LINK_LOST, srv,
                  "link lost");
  }
  if (current_mode() == req->mode) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_OK, srv,
                  "already in mode");
  }
  if (!set_mode_client_->service_is_ready()) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_LINK_LOST, srv,
                  "/mavros/set_mode not available");
  }

  auto request = std::make_shared<MavSetMode::Request>();
  request->custom_mode = req->mode;
  auto future = set_mode_client_->async_send_request(request);

  if (future.wait_for(std::chrono::duration<double>(command_timeout_s_)) !=
      std::future_status::ready) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_TIMEOUT, srv,
                  "no response from mavros");
  }
  if (!future.get()->mode_sent) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                  "mavros did not send mode");
  }

  const auto deadline = deadline_after(command_timeout_s_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (current_mode() == req->mode) {
      return finish(get_logger(), resp->error, ErrorCode::CODE_OK, srv,
                    "mode confirmed");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                "FC did not switch (check MAVProxy console)");
}

void VehicleNode::arm_cb(const std::shared_ptr<ArmSrv::Request> /*req*/,
                         std::shared_ptr<ArmSrv::Response> resp) {
  arm_disarm(true, resp->error);
}

void VehicleNode::disarm_cb(const std::shared_ptr<DisarmSrv::Request> /*req*/,
                            std::shared_ptr<DisarmSrv::Response> resp) {
  arm_disarm(false, resp->error);
}

// TODO: arm in a position mode (GUIDED/LOITER) while
// ready_for_position_control is false -> NOT_READY
void VehicleNode::arm_disarm(bool arm, ErrorCode &error) {
  const std::string srv = arm ? "arm" : "disarm";

  if (!link_ok()) {
    return finish(get_logger(), error, ErrorCode::CODE_LINK_LOST, srv,
                  "link lost");
  }
  if (is_armed() == arm) {
    return finish(get_logger(), error, ErrorCode::CODE_OK, srv,
                  arm ? "already armed" : "already disarmed");
  }
  if (!arming_client_->service_is_ready()) {
    return finish(get_logger(), error, ErrorCode::CODE_LINK_LOST, srv,
                  "/mavros/cmd/arming not available");
  }

  auto request = std::make_shared<MavCommandBool::Request>();
  request->value = arm;
  auto future = arming_client_->async_send_request(request);

  if (future.wait_for(std::chrono::duration<double>(command_timeout_s_)) !=
      std::future_status::ready) {
    return finish(get_logger(), error, ErrorCode::CODE_TIMEOUT, srv,
                  "no response from mavros");
  }
  const auto response = future.get();
  if (!response->success) {
    return finish(get_logger(), error, ErrorCode::CODE_REJECTED, srv,
                  "FC refused (MAV_RESULT " + std::to_string(response->result) +
                      ", check MAVProxy console)");
  }

  const auto deadline = deadline_after(command_timeout_s_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (is_armed() == arm) {
      return finish(get_logger(), error, ErrorCode::CODE_OK, srv, "confirmed");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return finish(get_logger(), error, ErrorCode::CODE_REJECTED, srv,
                "state did not change");
}

} // namespace drone_vehicle
