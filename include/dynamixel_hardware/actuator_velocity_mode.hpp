#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_VELOCITY_MODE_HPP

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorVelocityMode : public ActuatorMonitorMode {
public:
  ActuatorVelocityMode(double *const pos, double *const vel, double *const eff,
                       double *const vel_cmd)
      : ActuatorMonitorMode(pos, vel, eff), vel_cmd_(vel_cmd) {}

  virtual void starting() {
    ActuatorMonitorMode::starting();
    // TODO: switch to velocity mode
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    ActuatorMonitorMode::write(time, period);
    // TODO: write velocity command
  }

private:
  double *const vel_cmd_;
};
} // namespace dynamixel_hardware

#endif