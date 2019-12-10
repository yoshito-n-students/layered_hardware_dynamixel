#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_DISABLE_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_DISABLE_MODE_HPP

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorTorqueDisableMode : public ActuatorMonitorMode {
public:
  ActuatorTorqueDisableMode(double *const pos, double *const vel, double *const eff)
      : ActuatorMonitorMode(pos, vel, eff) {}

  virtual void starting() {
    ActuatorMonitorMode::starting();
    // TODO: torque disable
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    ActuatorMonitorMode::read(time, period);
  }
};
} // namespace dynamixel_hardware

#endif