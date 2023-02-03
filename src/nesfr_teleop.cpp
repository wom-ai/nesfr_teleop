#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

#include "nesfr_teleop/nesfr_teleop.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace nesfr_teleop
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link NesfrTeleop
 * directly into base nodes.
 */
struct NesfrTeleop::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  float linear_scale;
  float angular_scale;
};

/**
 * Constructs NesfrTeleop.
 */
NesfrTeleop::NesfrTeleop(const rclcpp::NodeOptions& options) : Node("nesfr_teleop_node", options)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
    std::bind(&NesfrTeleop::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->linear_scale = this->declare_parameter("linear_scale", 2.0f);
  pimpl_->angular_scale = this->declare_parameter("angular_scale", 2.0f);
}

NesfrTeleop::~NesfrTeleop()
{
  delete pimpl_;
}

void NesfrTeleop::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

  // Normalize and scale joystick input
  // NOTE: https://www.xarg.org/2017/07/how-to-map-a-square-to-a-circle/
  float &x = joy_msg->axes[1];
  float &y = joy_msg->axes[0];
  cmd_vel_msg->linear.x = x * sqrt(1 - pow(y, 2) / 2) * linear_scale;
  cmd_vel_msg->linear.y = y * sqrt(1 - pow(x, 2) / 2) * linear_scale;

  cmd_vel_msg->angular.z = (joy_msg->axes[4] - joy_msg->axes[5]) / 2 * angular_scale;

  cmd_vel_pub->publish(std::move(cmd_vel_msg));
}

void NesfrTeleop::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  sendCmdVelMsg(joy_msg);
}

}  // namespace nesfr_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(nesfr_teleop::NesfrTeleop)
