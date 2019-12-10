#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_NULL_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_NULL_MODE_HPP

#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorNullMode : public ActuatorOperatingModeBase {
public:
  ActuatorNullMode() {}

  virtual void starting() {
    // nothing to do
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
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