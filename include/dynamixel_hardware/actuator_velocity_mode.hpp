#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP

#include <cmath>
#include <limits>

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorVelocityMode : public ActuatorOperatingModeBase {
public:
  ActuatorVelocityMode(const ActuatorDataPtr &data) : ActuatorOperatingModeBase("velocity", data) {}

  virtual void starting() {
    // switch to velocity mode
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setVelocityControlMode);

    // set reasonable initial command
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readState(); }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeVelocityCommand();
      prev_vel_cmd_ = data_->vel_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  double prev_vel_cmd_;
};
} // namespace dynamixel_hardware

#endif