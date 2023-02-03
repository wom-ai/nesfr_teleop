#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nesfr_teleop/nesfr_teleop.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<nesfr_teleop::NesfrTeleop>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
