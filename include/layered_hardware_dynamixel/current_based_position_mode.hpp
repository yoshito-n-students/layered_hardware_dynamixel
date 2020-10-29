#ifndef LAYERED_HARDWARE_DYNAMIXEL_CURRENT_BASED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CURRENT_BASED_POSITION_MODE_HPP

#include <cmath>
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

class CurrentBasedPositionMode : public OperatingModeBase {
public:
  CurrentBasedPositionMode(const DynamixelActuatorDataPtr &data,
                           const std::map< std::string, int > &item_map)
      : OperatingModeBase("current_based_position", data), item_map_(item_map) {}

  virtual void starting() {
    // switch to current-based position mode
    enableOperatingMode(&DynamixelWorkbench::setCurrentBasedPositionControlMode);

    writeItems(item_map_);

    // use the present position as the initial position command
    readAllStates();
    data_->pos_cmd = data_->pos;
    data_->vel_cmd = 0.; // use velocity limit in the dynamixel's control table
    data_->eff_cmd = 0.; // use torque limit in the dynamixel's control table
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();

    cached_pos_ = boost::none;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read pos, vel, eff, etc
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write profile velocity if updated
    const bool do_write_vel(!std::isnan(data_->vel_cmd) &&
                            areNotEqual(data_->vel_cmd, prev_vel_cmd_));
    if (do_write_vel) {
      writeProfileVelocity();
      prev_vel_cmd_ = data_->vel_cmd;
    }

    // write effort limit if updated
    const bool do_write_eff(!std::isnan(data_->eff_cmd) &&
                            areNotEqual(data_->eff_cmd, prev_eff_cmd_));
    if (do_write_eff) {
      writeEffortCommand();
      prev_eff_cmd_ = data_->eff_cmd;
    }

    // if the profile velocity is 0, the user would want the actuator
    // to stop at the present position but 0 actually means unlimited.
    // to solve this mismatch, freeze the position command on that case.
    const bool do_freeze_pos(!std::isnan(data_->vel_cmd) &&
                             data_->dxl_wb->convertVelocity2Value(data_->id, data_->vel_cmd) == 0);
    if (do_freeze_pos) {
      if (!cached_pos_) {
        cached_pos_ = data_->pos;
      }
      data_->pos_cmd = cached_pos_.get();
    } else {
      cached_pos_ = boost::none;
    }

    // write goal position if the goal pos, profile velocity or effort limit have been updated
    // to make the change affect
    const bool do_write_pos(
        !std::isnan(data_->pos_cmd) &&
        (do_write_vel || do_write_eff || areNotEqual(data_->pos_cmd, prev_pos_cmd_)));
    if (do_write_pos) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }
  }

  virtual void stopping() { torqueOff(); }

private:
  const std::map< std::string, int > item_map_;
  double prev_pos_cmd_, prev_vel_cmd_, prev_eff_cmd_;
  boost::optional< double > cached_pos_;
};
} // namespace layered_hardware_dynamixel

#endif