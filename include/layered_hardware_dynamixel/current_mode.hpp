#ifndef LAYERED_HARDWARE_DYNAMIXEL_CURRENT_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CURRNET_MODE_HPP

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

class CurrentMode : public OperatingModeBase {
public:
  CurrentMode(const DynamixelActuatorDataPtr &data,
              const std::map<std::string, std::int32_t> &item_map)
      : OperatingModeBase("current", data), item_map_(item_map) {}

  virtual void starting() override {
    // switch to current mode
    enableOperatingMode(&DynamixelWorkbench::setCurrentControlMode);

    // set reasonable initial command
    data_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();

    readItems(&data_->additional_cmds);
    prev_additional_cmds_ = data_->additional_cmds;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    if (!std::isnan(data_->eff_cmd) && areNotEqual(data_->eff_cmd, prev_eff_cmd_)) {
      writeEffortCommand();
      prev_eff_cmd_ = data_->eff_cmd;
    }

    // write additional commands only when commands are updated
    for (const std::map<std::string, std::int32_t>::value_type &cmd : data_->additional_cmds) {
      std::int32_t &prev_cmd(prev_additional_cmds_[cmd.first]);
      if (cmd.second != prev_cmd) {
        writeItem(cmd.first, cmd.second);
        prev_cmd = cmd.second;
      }
    }
  }

  virtual void stopping() override { torqueOff(); }

private:
  const std::map<std::string, std::int32_t> item_map_;
  double prev_eff_cmd_;
  std::map<std::string, std::int32_t> prev_additional_cmds_;
};
} // namespace layered_hardware_dynamixel

#endif