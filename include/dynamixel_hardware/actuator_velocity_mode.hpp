#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP

#include <limits>
#include <map>
#include <string>

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorVelocityMode : public ActuatorOperatingModeBase {
public:
  ActuatorVelocityMode(const ActuatorDataPtr &data, const std::map< std::string, int > &item_map)
      : ActuatorOperatingModeBase("velocity", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to velocity mode
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setVelocityControlMode);

    writeItems(item_map_);
    
    // set reasonable initial command
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readState(); }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (isNotNaN(data_->vel_cmd) && areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeVelocityCommand();
      prev_vel_cmd_ = data_->vel_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  const std::map< std::string, int > item_map_;
  double prev_vel_cmd_;
};
} // namespace dynamixel_hardware

#endif