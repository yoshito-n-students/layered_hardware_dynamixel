#ifndef LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP

#include <limits>
#include <map>
#include <string>

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class VelocityMode : public OperatingModeBase {
public:
  VelocityMode(const DynamixelActuatorDataPtr &data, const std::map< std::string, int > &item_map)
      : OperatingModeBase("velocity", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to velocity mode
    enableOperatingMode(&DynamixelWorkbench::setVelocityControlMode);

    writeItems(item_map_);

    // set reasonable initial command
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readAllStates(); }

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
} // namespace layered_hardware_dynamixel

#endif