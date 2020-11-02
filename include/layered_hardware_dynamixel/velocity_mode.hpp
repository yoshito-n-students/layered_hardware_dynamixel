#ifndef LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_VELOCITY_MODE_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class VelocityMode : public OperatingModeBase {
public:
  VelocityMode(const DynamixelActuatorDataPtr &data,
               const std::map< std::string, std::int32_t > &item_map)
      : OperatingModeBase("velocity", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to velocity mode
    enableOperatingMode(&DynamixelWorkbench::setVelocityControlMode);

    writeItems(item_map_);

    // set reasonable initial command
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();

    readItems(&data_->additional_cmds);
    prev_additional_cmds_ = data_->additional_cmds;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readAllStates(); }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (!std::isnan(data_->vel_cmd) && areNotEqual(data_->vel_cmd, prev_vel_cmd_)) {
      writeVelocityCommand();
      prev_vel_cmd_ = data_->vel_cmd;
    }

    // write additional commands only when commands are updated
    for (const std::map< std::string, std::int32_t >::value_type &cmd : data_->additional_cmds) {
      std::int32_t &prev_cmd(prev_additional_cmds_[cmd.first]);
      if (cmd.second != prev_cmd) {
        writeItem(cmd.first, cmd.second);
        prev_cmd = cmd.second;
      }
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  const std::map< std::string, std::int32_t > item_map_;
  double prev_vel_cmd_;
  std::map< std::string, std::int32_t > prev_additional_cmds_;
};
} // namespace layered_hardware_dynamixel

#endif