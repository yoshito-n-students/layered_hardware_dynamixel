#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_MODE_HPP

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorTorqueMode : public ActuatorMonitorMode {
public:
  ActuatorTorqueMode(double *const pos, double *const vel, double *const eff, double *const eff_cmd)
      : ActuatorMonitorMode(pos, vel, eff), eff_cmd_(eff_cmd) {}

  virtual void starting() {
    ActuatorMonitorMode::starting();
    // TODO: switch to current mode
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    ActuatorMonitorMode::write(time, period);
    // TODO: write current command
  }

private:
  double *const eff_cmd_;
};
} // namespace dynamixel_hardware

#endif