#include "drone_vehicle/vehicle_node.hpp"
#include <exception>
#include <memory>
#include <rclcpp/logging.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<drone_vehicle::VehicleNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
