#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_EXTENDED_POSITION_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_EXTENDED_POSITION_MODE_HPP

#include <limits>

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorExtendedPositionMode : public ActuatorOperatingModeBase {
public:
  ActuatorExtendedPositionMode(const ActuatorDataPtr &data)
      : ActuatorOperatingModeBase("extended_position", data) {}

  virtual void starting() {
    // switch to extended-position mode & torque enable
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setExtendedPositionControlMode);

    // read present position & use it as the initial position command
    // TODO: instead, read goal position on dynamixel
    //       and use it as the initial & prev position commands
    readPosition();
    data_->pos_cmd = data_->pos;
    data_->vel_cmd = 0.; // use velocity limit in the dynamixel's control table
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel, & eff
    readState();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // send position command if updated
    if (areNotEqual(data_->pos_cmd, prev_pos_cmd_)) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }
    if (areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeProfileVelocity();
      prev_vel_cmd_ = data_->vel_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  double prev_pos_cmd_, prev_vel_cmd_;
};
} // namespace dynamixel_hardware

#endif