#ifndef LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP

#include <limits>
#include <map>
#include <string>

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class ExtendedPositionMode : public OperatingModeBase {
public:
  ExtendedPositionMode(const DynamixelActuatorDataPtr &data,
                       const std::map< std::string, int > &item_map)
      : OperatingModeBase("extended_position", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to extended-position mode & torque enable
    setOperatingModeAndTorqueOn(&DynamixelWorkbench::setExtendedPositionControlMode);

    writeItems(item_map_);

    // use the present position as the initial command
    readState();
    data_->pos_cmd = data_->pos;
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel, & eff
    readState();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // send position command if updated
    if (isNotNaN(data_->vel_cmd) && areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeProfileVelocity();
      prev_vel_cmd_ = data_->vel_cmd;
    }
    if (isNotNaN(data_->pos_cmd) && areNotEqual(data_->pos_cmd, prev_pos_cmd_)) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  const std::map< std::string, int > item_map_;
  double prev_pos_cmd_, prev_vel_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif