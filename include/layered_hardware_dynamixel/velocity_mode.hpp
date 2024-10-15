#ifndef LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class VelocityMode : public OperatingModeInterface {
public:
  VelocityMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("velocity", context) {}

  virtual void starting() override {
    // switch to velocity mode
    enable_operating_mode(context_, &DynamixelWorkbench::setVelocityControlMode);

    // set reasonable initial command
    context_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    read_all_states(context_);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->vel_cmd) && !bitwise_equal(context_->vel_cmd, prev_vel_cmd_)) {
      write_velocity_command(context_);
      prev_vel_cmd_ = context_->vel_cmd;
    }
  }

  virtual void stopping() override { torque_off(context_); }

private:
  double prev_vel_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif