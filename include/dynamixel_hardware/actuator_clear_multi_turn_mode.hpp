#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_CLEAR_MULTI_TURN_MODE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_CLEAR_MULTI_TURN_MODE_HPP

#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class ActuatorClearMultiTurnMode : public ActuatorOperatingModeBase {
public:
  ActuatorClearMultiTurnMode(const ActuatorDataPtr &data)
      : ActuatorOperatingModeBase("clear_multi_turn", data) {}

  virtual void starting() {
    if (!data_->dxl_wb.clearMultiTurn(data_->id)) {
      ROS_ERROR_STREAM("ActuatorClearMultiTurnMode::starting(): Failed to clear multi turn offset "
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
} // namespace dynamixel_hardware

#endif