#ifndef LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_BASE_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_dynamixel {

class OperatingModeBase {
public:
  OperatingModeBase(const std::string &name, const DynamixelActuatorDataPtr &data)
      : name_(name), data_(data) {}

  virtual ~OperatingModeBase() {}

  std::string getName() const { return name_; }

  // TODO: retrun bool to inform result of mode switching to the upper class
  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  //
  // instruction functions for chiled classes
  //

  bool ping() {
    const char *log(NULL);
    if (!data_->dxl_wb->ping(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::ping(): Failed to ping to '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::ping()"));
      return false;
    }
    return true;
  }

  bool pingFor(const ros::Duration &timeout) {
    const ros::Time timeout_abs(ros::Time::now() + timeout);
    while (true) {
      if (ros::Time::now() > timeout_abs) {
        ROS_ERROR_STREAM("OperatingModeBase::pingFor(): No ping response from '"
                         << data_->name << "' (id: " << static_cast< int >(data_->id) << ") for "
                         << timeout.toSec() << " s");
        return false;
      }
      if (data_->dxl_wb->ping(data_->id)) {
        return true;
      }
    }
    // never reach here
  }

  bool reboot() {
    // instruct rebooting
    const char *log(NULL);
    if (!data_->dxl_wb->reboot(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::reboot(): Failed to reboot '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::reboot()"));
      return false;
    }
    return true;
  }

  //
  // read functions for chiled classes
  //

  bool readItem(const std::string &item, int32_t *value) {
    const char *log(NULL);
    if (!data_->dxl_wb->itemRead(data_->id, item.c_str(), value, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::readItem(): Failed to read control table item '"
                       << item << "' of '" << data_->name
                       << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::itemRead()"));
      return false;
    }
    return true;
  }

  bool readItems(std::map< std::string, std::int32_t > *const items) {
    bool result(true);
    for (std::map< std::string, std::int32_t >::value_type &item : *items) {
      if (!readItem(item.first, &item.second)) {
        result = false;
      }
    }
    return result;
  }

  bool readPosition() {
    float rad;
    const char *log(NULL);
    if (!data_->dxl_wb->getRadian(data_->id, &rad, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::readPosition(): Failed to read position from '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::getRadian()"));
      return false;
    }
    data_->pos = rad;
    return true;
  }

  bool readVelocity() {
    std::int32_t value;
    // As of dynamixel_workbench_toolbox v2.0.0,
    // DynamixelWorkbench::getVelocity() reads a wrong item ...
    if (!readItem("Present_Velocity", &value)) {
      return false;
    }
    data_->vel = data_->dxl_wb->convertValue2Velocity(data_->id, value);
    return true;
  }

  bool readEffort() {
    std::int32_t value;
    if (!readItem("Present_Current", &value)) {
      return false;
    }
    // TODO: use new API once supported
    data_->eff = data_->dxl_wb->convertValue2Current(/* data_->id,*/ value) *
                 data_->torque_constant / 1000.0;
    return true;
  }

  bool readAdditionalStates() {
    bool result(true);
    for (std::map< std::string, std::int32_t >::value_type &state : data_->additional_states) {
      if (!readItem(state.first, &state.second)) {
        result = false;
      }
    }
    return result;
  }

  bool readAllStates() {
    // if one fails, "return readPosition() && readVelocity() && ..." does not call others.
    // on the other hand, lines below call all anyway to read info as much as possible.
    const bool pos_result(readPosition());
    const bool vel_result(readVelocity());
    const bool eff_result(readEffort());
    const bool additional_result(readAdditionalStates());
    return pos_result && vel_result && eff_result && additional_result;
  }

  //
  // write functions for child classes
  //

  bool enableOperatingMode(bool (DynamixelWorkbench::*const set_func)(std::uint8_t,
                                                                      const char **)) {
    const char *log;
    // disable torque to make the actuator ready to change operating modes
    log = NULL;
    if (!data_->dxl_wb->torqueOff(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::enableOperatingMode(): Failed to disable torque of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
      return false;
    }
    // change operating modes
    log = NULL;
    if (!(data_->dxl_wb->*set_func)(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::enableOperatingMode(): Failed to set operating mode of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench"));
      return false;
    }
    // activate new operating mode by enabling torque
    log = NULL;
    if (!data_->dxl_wb->torqueOn(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::enableOperatingMode(): Failed to enable torque of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::torqueOn()"));
      return false;
    }
    return true;
  }

  bool torqueOff() {
    const char *log(NULL);
    if (!data_->dxl_wb->torqueOff(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::torqueOff(): Failed to disable torque of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::torqueOff()"));
      return false;
    }
    return true;
  }

  bool clearMultiTurn() {
    const char *log(NULL);
    if (!data_->dxl_wb->clearMultiTurn(data_->id, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::clearMultiTurn(): Failed to clear multi turn count of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << "): "
                       << (log ? log : "No log from DynamixelWorkbench::clearMultiTurn()"));
      return false;
    }
    return true;
  }

  bool writeItem(const std::string &item, const std::int32_t value) {
    const char *log(NULL);
    if (!data_->dxl_wb->itemWrite(data_->id, item.c_str(), value, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::writeItem(): Failed to set control table item '"
                       << item << "' of '" << data_->name
                       << "' (id: " << static_cast< int >(data_->id) << " to " << value << ": "
                       << (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
      return false;
    }
    return true;
  }

  bool writeItems(const std::map< std::string, std::int32_t > &item_map) {
    for (const std::map< std::string, std::int32_t >::value_type &item : item_map) {
      if (!writeItem(item.first, item.second)) {
        return false;
      }
    }
    return true;
  }

  bool writePositionCommand() {
    const float cmd(static_cast< float >(data_->pos_cmd));
    const char *log(NULL);
    if (!data_->dxl_wb->goalPosition(data_->id, cmd, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::writePositionCommand(): Failed to set goal position of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << ") to "
                       << cmd << ": "
                       << (log ? log : "No log from DynamixelWorkbench::goalPosition()"));
      return false;
    }
    return true;
  }

  bool writeVelocityCommand() {
    const float cmd(static_cast< float >(data_->vel_cmd));
    const char *log(NULL);
    if (!data_->dxl_wb->goalVelocity(data_->id, cmd, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::writeVelocityCommand(): Failed to set goal velocity of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << ") to "
                       << cmd << ": "
                       << (log ? log : "No log from DynamixelWorkbench::goalVelocity()"));
      return false;
    }
    return true;
  }

  bool writeProfileVelocity() {
    const float cmd(static_cast< float >(std::abs(data_->vel_cmd)));
    const std::int32_t cmd_value(data_->dxl_wb->convertVelocity2Value(data_->id, cmd));
    return writeItem("Profile_Velocity", cmd_value);
  }

  bool writeEffortCommand() {
    const float cmd(data_->eff_cmd / data_->torque_constant * 1000.0);
    // TODO: use new API once supported
    const std::int16_t cmd_value(data_->dxl_wb->convertCurrent2Value(
        /* data_->id, */ cmd));
    return writeItem("Goal_Current", cmd_value);
  }

  //
  // utility
  //

  static bool areNotEqual(const double a, const double b) {
    // does !(|a - b| < EPS) instead of (|a - b| >= EPS) to return True when a and/or b is NaN
    return !(std::abs(a - b) < std::numeric_limits< double >::epsilon());
  }

protected:
  const std::string name_;
  const DynamixelActuatorDataPtr data_;
};

typedef std::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef std::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;
} // namespace layered_hardware_dynamixel

#endif