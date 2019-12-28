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
    reboot();
    // confirm the actuator has been rebooted by ping for certain duration
    pingFor(ros::Duration(0.5));
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