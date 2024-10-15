#ifndef LAYERED_HARDWARE_DYNAMIXEL_CLEAR_MULTI_TURN_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CLEAR_MULTI_TURN_MODE_HPP

#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class ClearMultiTurnMode : public OperatingModeInterface {
public:
  ClearMultiTurnMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("clear_multi_turn", context) {}

  virtual void starting() override { clear_multi_turn(context_); }

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