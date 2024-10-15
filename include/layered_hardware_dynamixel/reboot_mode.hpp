#ifndef LAYERED_HARDWARE_DYNAMIXEL_REBOOT_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_REBOOT_MODE_HPP

#include <chrono>
#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class RebootMode : public OperatingModeInterface {
public:
  RebootMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("reboot", context) {}

  virtual void starting() override {
    reboot(context_);
    // confirm the actuator has been rebooted by ping for certain duration
    ping_for(context_, rclcpp::Duration(std::chrono::milliseconds(500)));
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do
  }

  virtual void stopping() override {
    // nothing to do
  }
};
} // namespace layered_hardware_dynamixel

#endif