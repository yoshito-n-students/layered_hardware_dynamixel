#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_REBOOT_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_REBOOT_MODE_HPP

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/time.h>

#include <dynamixel/instruction_packet.hpp>
#include <dynamixel/instructions/reboot.hpp>
#include <dynamixel/protocols/protocol2.hpp>

namespace dynamixel_hardware {

class ActuatorRebootMode : public ActuatorOperatingModeBase {
public:
  ActuatorRebootMode(const ActuatorDataPtr &data) : ActuatorOperatingModeBase("reboot", data) {}

  virtual void starting() {
    try {
      data_->device.send(di::Reboot< dp::Protocol2 >(data_->servo->id()));
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM("ActuatorRebootMode::starting(): Failed to reboot "
                       << data_->name << " (id: " << data_->servo->id() << "): " << err.msg());
      return;
    }

    // TODO: confirm reset by ping ??
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