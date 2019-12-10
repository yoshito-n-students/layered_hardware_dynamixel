#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_MONITOR_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_MONITOR_MODE_HPP

#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorMonitorMode : public ActuatorOperatingModeBase {
public:
  ActuatorMonitorMode(double *const pos, double *const vel, double *const eff)
      : pos_(pos), vel_(vel), eff_(eff) {}

  virtual void starting() {
    // nothing to do
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // TODO: read states
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void stopping() {
    // nothing to do
  }

private:
  double *const pos_, *vel_, *eff_;
};
} // namespace dynamixel_hardware

#endif