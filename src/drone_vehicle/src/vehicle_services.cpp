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
    RCLCPP_INFO(logger, "%s : %s", service.c_str(), why.c_str());
  } else {
    RCLCPP_WARN(logger, "%s : %s", service.c_str(), why.c_str());
  }
}
} // namespace

void VehicleNode::set_mode_cb(const std::shared_ptr<SetModeSrv::Request> req,
                              std::shared_ptr<SetModeSrv::Response> resp) {
  const std::string srv = "set mode " + req->mode;

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
                  "mode already selected");
  }

  if (!set_mode_client_->service_is_ready()) {

    return finish(get_logger(), resp->error, ErrorCode::CODE_LINK_LOST, srv,
                  "service not ready");
  }

  auto request = std::make_shared<MavSetMode::Request>();
  request->custom_mode = req->mode;
  auto future = set_mode_client_->async_send_request(request);

  const auto timeout = std::chrono::duration<double>(command_timeout_s_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                  "no response from mavros");
  }

  if (!future.get()->mode_sent) {
    return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                  "mavros didnt send mode");
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (current_mode() == req->mode) {
      return finish(get_logger(), resp->error, ErrorCode::CODE_OK, srv,
                    "mode confirmed");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return finish(get_logger(), resp->error, ErrorCode::CODE_REJECTED, srv,
                "Fc did not switch");
}
} // namespace drone_vehicle
