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

    // use the present position as the initial command
    data_->pos_cmd = data_->pos;
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel, & eff
    readState();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // send position command if updated
    if (isNotNaN(data_->pos_cmd) && areNotEqual(data_->pos_cmd, prev_pos_cmd_)) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }
    if (isNotNaN(data_->vel_cmd) && areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
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