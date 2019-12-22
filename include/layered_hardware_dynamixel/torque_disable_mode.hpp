#ifndef LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP

#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class TorqueDisableMode : public OperatingModeBase {
public:
  TorqueDisableMode(const DynamixelActuatorDataPtr &data)
      : OperatingModeBase("torque_disable", data) {}

  virtual void starting() { torqueOff(); }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel & eff
    readState();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void stopping() {
    // nothing to do
  }
};
} // namespace layered_hardware_dynamixel

#endif