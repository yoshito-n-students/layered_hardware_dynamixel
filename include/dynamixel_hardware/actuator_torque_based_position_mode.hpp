#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_BASED_POSITION_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_TORQUE_BASED_POSITION_MODE_HPP

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorTorqueBasedPositionMode : public ActuatorMonitorMode {
public:
  ActuatorTorqueBasedPositionMode(double *const pos, double *const vel, double *const eff,
                                  double *const pos_cmd, double *const vel_cmd,
                                  double *const eff_cmd)
      : ActuatorMonitorMode(pos, vel, eff), pos_cmd_(pos_cmd), vel_cmd_(vel_cmd),
        eff_cmd_(eff_cmd) {}

  virtual void starting() {
    ActuatorMonitorMode::starting();
    // TODO: switch to current-based position mode
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    ActuatorMonitorMode::write(time, period);
    // TODO: write commands
  }

private:
  double *const pos_cmd_, *const vel_cmd_, *const eff_cmd_;
};
} // namespace dynamixel_hardware

#endif