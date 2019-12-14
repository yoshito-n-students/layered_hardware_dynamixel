#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_CURRENT_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_CURRNET_MODE_HPP

#include <limits>

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorCurrentMode : public ActuatorOperatingModeBase {
public:
  ActuatorCurrentMode(const ActuatorDataPtr &data) : ActuatorOperatingModeBase("current", data) {}

  virtual void starting() {
    // switch to current mode
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setCurrentControlMode);

    // set reasonable initial command
    data_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readState(); }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (areNotEqual(data_->eff_cmd, prev_eff_cmd_)) {
      writeEffortCommand();
      prev_eff_cmd_ = data_->eff_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  double prev_eff_cmd_;
};
} // namespace dynamixel_hardware

#endif