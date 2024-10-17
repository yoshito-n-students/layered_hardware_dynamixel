#ifndef LAYERED_HARDWARE_DYNAMIXEL_CURRENT_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CURRNET_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class CurrentMode : public OperatingModeInterface {
public:
  CurrentMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("current", context) {}

  virtual void starting() override {
    // switch to current mode
    enable_operating_mode(context_, &DynamixelWorkbench::setCurrentControlMode);

    // set reasonable initial command
    context_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    read_all_states(context_);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!std::isnan(context_->eff_cmd) && context_->eff_cmd != prev_eff_cmd_) {
      write_effort_command(context_);
      prev_eff_cmd_ = context_->eff_cmd;
    }
  }

  virtual void stopping() override { torque_off(context_); }

private:
  double prev_eff_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif