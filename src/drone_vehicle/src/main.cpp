#include "drone_vehicle/vehicle_node.hpp"
#include <exception>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<drone_vehicle::VehicleNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
