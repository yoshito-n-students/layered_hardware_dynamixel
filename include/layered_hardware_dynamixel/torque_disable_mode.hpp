#ifndef LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_TORQUE_DISABLE_MODE_HPP

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class TorqueDisableMode : public OperatingModeBase {
public:
  TorqueDisableMode(const DynamixelActuatorDataPtr &data)
      : OperatingModeBase("torque_disable", data) {}

  virtual void prepareStart() override {
    // nothing to do
  }
  
  virtual void starting() override { torqueOff(); }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, etc
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // nothing to do
  }

  virtual void stopping() override {
    // nothing to do
  }
};
} // namespace layered_hardware_dynamixel

#endif