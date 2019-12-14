#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_CURRENT_BASED_POSITION_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_CURRENT_BASED_POSITION_MODE_HPP

#include <limits>

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorCurrentBasedPositionMode : public ActuatorOperatingModeBase {
public:
  ActuatorCurrentBasedPositionMode(const ActuatorDataPtr &data)
      : ActuatorOperatingModeBase("current_based_position", data) {}

  virtual void starting() {
    // switch to current-based position mode
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setCurrentBasedPositionControlMode);

    // read present position & use it as the initial position command
    // TODO: instead, read goal position on dynamixel
    //       and use it as the initial & prev position commands
    readPosition();
    data_->pos_cmd = data_->pos;
    data_->vel_cmd = 0.; // use velocity limit in the dynamixel's control table
    data_->eff_cmd = 0.; // use torque limit in the dynamixel's control table
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel & eff
    readState();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write commands
    if (areNotEqual(data_->pos_cmd, prev_pos_cmd_)) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }
    if (areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeProfileVelocity();
      prev_vel_cmd_ = data_->vel_cmd;
    }
    if (areNotEqual(data_->eff_cmd, prev_eff_cmd_)) {
      writeEffortCommand();
      prev_eff_cmd_ = data_->eff_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  double prev_pos_cmd_, prev_vel_cmd_, prev_eff_cmd_;
};
} // namespace dynamixel_hardware

#endif