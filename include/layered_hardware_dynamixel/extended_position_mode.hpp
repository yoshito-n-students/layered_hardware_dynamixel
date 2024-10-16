#ifndef LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class ExtendedPositionMode : public OperatingModeInterface {
public:
  ExtendedPositionMode(const std::shared_ptr<DynamixelActuatorContext> &context)
      : OperatingModeInterface("extended_position", context) {}

  virtual void starting() override {
    // switch to extended-position mode & torque enable
    enable_operating_mode(context_, &DynamixelWorkbench::setExtendedPositionControlMode);

    // use the present position as the initial command
    read_all_states(context_);
    context_->pos_cmd = context_->pos;
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
    context_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits<double>::quiet_NaN();

    cached_pos_ = std::nullopt;
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // read pos, vel, eff, etc
    read_all_states(context_);
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // write profile velocity if updated
    const bool do_write_vel(!std::isnan(context_->vel_cmd) &&
                            !bitwise_equal(context_->vel_cmd, prev_vel_cmd_));
    if (do_write_vel) {
      write_profile_velocity(context_);
      prev_vel_cmd_ = context_->vel_cmd;
    }

    // if the profile velocity is 0, the user would want the actuator
    // to stop at the present position but 0 actually means unlimited.
    // to solve this mismatch, freeze the position command on that case.
    const bool do_freeze_pos(
        !std::isnan(context_->vel_cmd) &&
        context_->dxl_wb->convertVelocity2Value(context_->id, context_->vel_cmd) == 0);
    if (do_freeze_pos) {
      if (!cached_pos_) {
        cached_pos_ = context_->pos;
      }
      context_->pos_cmd = cached_pos_.value();
    } else {
      cached_pos_ = std::nullopt;
    }

    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    const bool do_write_pos(!std::isnan(context_->pos_cmd) &&
                            (do_write_vel || !bitwise_equal(context_->pos_cmd, prev_pos_cmd_)));
    if (do_write_pos) {
      write_position_command(context_);
      prev_pos_cmd_ = context_->pos_cmd;
    }
  }

  virtual void stopping() override { torque_off(context_); }

private:
  double prev_pos_cmd_, prev_vel_cmd_;
  std::optional<double> cached_pos_;
};
} // namespace layered_hardware_dynamixel

#endif