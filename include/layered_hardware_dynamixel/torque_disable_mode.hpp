#ifndef LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP

#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class TorqueDisableMode : public OperatingModeInterface {
public:
  TorqueDisableMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("torque_disable", context) {}

  virtual void starting() override { torque_off(context_); }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    read_all_states(context_);
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