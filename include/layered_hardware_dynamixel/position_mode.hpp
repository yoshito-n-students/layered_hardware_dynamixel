#ifndef LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_POSITION_MODE_HPP

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

#include <boost/optional.hpp>

namespace layered_hardware_dynamixel {

class PositionMode : public OperatingModeBase {
public:
  PositionMode(const DynamixelActuatorDataPtr &data,
               const std::map<std::string, std::int32_t> &item_map)
      : OperatingModeBase("position", data), item_map_(item_map) {}

  virtual void starting() override {
    // switch to position mode & torque enable
    enableOperatingMode(&DynamixelWorkbench::setPositionControlMode);

    writeItems(item_map_);

    // use the present position as the initial command
    readAllStates();
    data_->pos_cmd = data_->pos;
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();

    readItems(&data_->additional_cmds);
    prev_additional_cmds_ = data_->additional_cmds;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, etc
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    const bool do_write_pos(!std::isnan(data_->pos_cmd) &&
                            areNotEqual(data_->pos_cmd, prev_pos_cmd_));
    if (do_write_pos) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }

    // write additional commands only when commands are updated
    for (const std::map<std::string, std::int32_t>::value_type &cmd : data_->additional_cmds) {
      std::int32_t &prev_cmd(prev_additional_cmds_[cmd.first]);
      const bool do_write_cmd(cmd.second != prev_cmd);
      if (do_write_cmd) {
        writeItem(cmd.first, cmd.second);
        prev_cmd = cmd.second;
      }
    }
  }

  virtual void stopping() override { torqueOff(); }

private:
  const std::map<std::string, std::int32_t> item_map_;
  double prev_pos_cmd_;
  std::map<std::string, std::int32_t> prev_additional_cmds_;
};
} // namespace layered_hardware_dynamixel

#endif