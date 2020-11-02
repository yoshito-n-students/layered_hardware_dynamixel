#ifndef LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_EXTENDED_POSITION_MODE_HPP

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

class ExtendedPositionMode : public OperatingModeBase {
public:
  ExtendedPositionMode(const DynamixelActuatorDataPtr &data,
                       const std::map< std::string, std::int32_t > &item_map)
      : OperatingModeBase("extended_position", data), item_map_(item_map) {}

  virtual void prepareStart() override {
    // fetch initial value of commands here
    // to make them valid before executing a controller's starting()
    readAllStates();
    readItems(&data_->additional_cmds);
  }
  
  virtual void starting() override {
    // switch to extended-position mode & torque enable
    enableOperatingMode(&DynamixelWorkbench::setExtendedPositionControlMode);

    writeItems(item_map_);

    // use the present position as the initial command
    data_->pos_cmd = data_->pos;
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
    data_->vel_cmd = 0.;
    prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();
    prev_additional_cmds_ = data_->additional_cmds;

    cached_pos_ = boost::none;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read pos, vel, eff, etc
    readAllStates();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write profile velocity if updated
    const bool do_write_vel(!std::isnan(data_->vel_cmd) &&
                            areNotEqual(data_->vel_cmd, prev_vel_cmd_));
    if (do_write_vel) {
      writeProfileVelocity();
      prev_vel_cmd_ = data_->vel_cmd;
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

    // write goal position if the goal pos or profile velocity have been updated
    // to make the change affect
    const bool do_write_pos(!std::isnan(data_->pos_cmd) &&
                            (do_write_vel || areNotEqual(data_->pos_cmd, prev_pos_cmd_)));
    if (do_write_pos) {
      writePositionCommand();
      prev_pos_cmd_ = data_->pos_cmd;
    }

    // write additional commands only when commands are updated
    for (const std::map< std::string, std::int32_t >::value_type &cmd : data_->additional_cmds) {
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
  const std::map< std::string, std::int32_t > item_map_;
  double prev_pos_cmd_, prev_vel_cmd_;
  std::map< std::string, std::int32_t > prev_additional_cmds_;
  boost::optional< double > cached_pos_;
};
} // namespace layered_hardware_dynamixel

#endif