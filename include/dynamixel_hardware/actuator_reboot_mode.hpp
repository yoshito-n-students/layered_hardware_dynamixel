#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_REBOOT_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_REBOOT_MODE_HPP

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorRebootMode : public ActuatorOperatingModeBase {
public:
  ActuatorRebootMode(const ActuatorDataPtr &data) : ActuatorOperatingModeBase("reboot", data) {}

  virtual void starting() {
    if (!data_->dxl_wb.reboot(data_->id)) {
      ROS_ERROR_STREAM("ActuatorRebootMode::starting(): Failed to reboot "
                       << data_->name << " (id: " << data_->id << ")");
    }
    // TODO: ping to wait rebooted ??
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