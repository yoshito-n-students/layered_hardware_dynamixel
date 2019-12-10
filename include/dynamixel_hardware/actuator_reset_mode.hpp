#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_RESET_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_RESET_MODE_HPP

#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorResetMode : public ActuatorOperatingModeBase {
public:
  ActuatorResetMode() {}

  virtual void starting() {
    // TODO: reset actuator
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void stoping() {
    // nothing to do
  }
};
} // namespace dynamixel_hardware

#endif