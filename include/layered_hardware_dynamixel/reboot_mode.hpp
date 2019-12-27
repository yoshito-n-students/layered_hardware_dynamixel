#ifndef LAYERED_HARDWARE_DYNAMIXEL_REBOOT_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_REBOOT_MODE_HPP

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class RebootMode : public OperatingModeBase {
public:
  RebootMode(const DynamixelActuatorDataPtr &data) : OperatingModeBase("reboot", data) {}

  virtual void starting() {
    // instruct rebooting
    if (!data_->dxl_wb->reboot(data_->id)) {
      ROS_ERROR_STREAM("RebootMode::starting(): Failed to reboot the actuator '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << ")");
      return;
    }
    // wait rebooted
    const ros::Time reboot_time(ros::Time::now());
    while (true) {
      // return on successful ping
      if (data_->dxl_wb->ping(data_->id)) {
        ROS_INFO_STREAM("RebootMode::starting(): Actuator '"
                        << data_->name << "' (id: " << static_cast< int >(data_->id)
                        << ") was successfully rebooted");
        return;
      }
      // or timeout
      if ((ros::Time::now() - reboot_time).toSec() > 0.5) {
        ROS_ERROR_STREAM("RebootMode::starting(): No ping response after reboot from the actuator '"
                         << data_->name << "' (id: " << static_cast< int >(data_->id) << ")");
        return;
      }
    }
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
} // namespace layered_hardware_dynamixel

#endif