#ifndef LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class PositionMode : public OperatingModeInterface {
public:
  PositionMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("position", context) {}

  virtual void starting() override {
    // switch to position mode & torque enable
    enable_operating_mode(context_, &DynamixelWorkbench::setPositionControlMode);

    // use the present position as the initial command
    read_all_states(context_);
    context_->pos_cmd = context_->pos;
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    read_all_states(context_);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    if (!std::isnan(context_->pos_cmd) && context_->pos_cmd != prev_pos_cmd_) {
      write_position_command(context_);
      prev_pos_cmd_ = context_->pos_cmd;
    }
  }

  virtual void stopping() override { torque_off(context_); }

private:
  double prev_pos_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif