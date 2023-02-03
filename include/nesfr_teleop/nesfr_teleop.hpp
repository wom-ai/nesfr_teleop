#ifndef NESFR_TELEOP_NESFR_TELEOP_H
#define NESFR_TELEOP_NESFR_TELEOP_H

#include <rclcpp/rclcpp.hpp>

namespace nesfr_teleop
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class NesfrTeleop : public rclcpp::Node
{
public:
  explicit NesfrTeleop(const rclcpp::NodeOptions& options);

  virtual ~NesfrTeleop();

private:
  struct Impl;
  Impl* pimpl_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle;  
};

}  // namespace nesfr_teleop

#endif  // NESFR_TELEOP_NESFR_TELEOP_H
