#ifndef LAYERED_HARDWARE_DYNAMIXEL_CLEAR_MULTI_TURN_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CLEAR_MULTI_TURN_MODE_HPP

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class ClearMultiTurnMode : public OperatingModeBase {
public:
  ClearMultiTurnMode(const DynamixelActuatorDataPtr &data)
      : OperatingModeBase("clear_multi_turn", data) {}

  virtual void starting() {
    if (!data_->dxl_wb.clearMultiTurn(data_->id)) {
      ROS_ERROR_STREAM("ClearMultiTurnMode::starting(): Failed to clear multi turn offset "
                       << data_->name << " (id: " << data_->id << ")");
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