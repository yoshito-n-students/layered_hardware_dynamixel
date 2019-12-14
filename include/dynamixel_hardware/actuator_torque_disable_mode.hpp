#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_DISABLE_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_DISABLE_MODE_HPP

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorTorqueDisableMode : public ActuatorOperatingModeBase {
public:
  ActuatorTorqueDisableMode(const ActuatorDataPtr &data)
      : ActuatorOperatingModeBase("torque_disable", data) {}

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
} // namespace dynamixel_hardware

#endif