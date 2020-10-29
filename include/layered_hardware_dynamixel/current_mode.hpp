#ifndef LAYERED_HARDWARE_DYNAMIXEL_CURRENT_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CURRNET_MODE_HPP

#include <cmath>
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
  CurrentMode(const DynamixelActuatorDataPtr &data, const std::map< std::string, int > &item_map)
      : OperatingModeBase("current", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to current mode
    enableOperatingMode(&DynamixelWorkbench::setCurrentControlMode);

    // set reasonable initial command
    data_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) { readAllStates(); }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (!std::isnan(data_->eff_cmd) && areNotEqual(data_->eff_cmd, prev_eff_cmd_)) {
      writeEffortCommand();
      prev_eff_cmd_ = data_->eff_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  const std::map< std::string, int > item_map_;
  double prev_eff_cmd_;
};
} // namespace layered_hardware_dynamixel

#endif