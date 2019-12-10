#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_POSITION_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_POSITION_MODE_HPP

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorPositionMode : public ActuatorMonitorMode {
public:
  ActuatorPositionMode(double *const pos, double *const vel, double *const eff,
                       double *const pos_cmd)
      : ActuatorMonitorMode(pos, vel, eff), pos_cmd_(pos_cmd) {}

  virtual void starting() {
    ActuatorMonitorMode::starting();
    // TODO: switch to multi-turn mode
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    ActuatorMonitorMode::write(time, period);
    // TODO: write position command
  }

private:
  double *const pos_cmd_;
};
} // namespace dynamixel_hardware

#endif