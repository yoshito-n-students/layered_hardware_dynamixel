#ifndef LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_BASE_HPP

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for isnan()
#include <boost/shared_ptr.hpp>

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
    int32_t value;
    const char *log(NULL);
    // As of dynamixel_workbench_toolbox v2.0.0,
    // DynamixelWorkbench::getVelocity() reads a wrong item ...
    if (!data_->dxl_wb->itemRead(data_->id, "Present_Velocity", &value, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::readVelocity(): Failed to read velocity from '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::itemRead()"));
      return false;
    }
    data_->vel = data_->dxl_wb->convertValue2Velocity(data_->id, value);
    return true;
  }

  bool readEffort() {
    int32_t value;
    const char *log(NULL);
    if (!data_->dxl_wb->itemRead(data_->id, "Present_Current", &value, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::readEffort(): Failed to read current from '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id)
                       << "): " << (log ? log : "No log from DynamixelWorkbench::itemRead()"));
      return false;
    }
    // TODO: use new API once supported
    data_->eff = data_->dxl_wb->convertValue2Current(/* data_->id,*/ value) *
                 data_->torque_constant / 1000.0;
    return true;
  }

  bool readAdditionalStates() {
    typedef std::map< std::string, hie::ByteArray > StateMap;
    bool result(true);
    BOOST_FOREACH (StateMap::value_type &state, data_->additional_states) {
      int32_t value;
      const char *log(NULL);
      if (data_->dxl_wb->itemRead(data_->id, state.first.c_str(), &value, &log)) {
        state.second = hie::ByteArray::from(value);
      } else {
        ROS_ERROR_STREAM("OperatingModeBase::readAdditionalStates(): Failed to read '"
                         << state.first << "' from '" << data_->name
                         << "' (id: " << static_cast< int >(data_->id)
                         << "): " << (log ? log : "No log from DynamixelWorkbench::itemRead()"));
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

  bool enableOperatingMode(bool (DynamixelWorkbench::*const set_func)(uint8_t, const char **)) {
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

  bool writeItems(const std::map< std::string, int > &item_map) {
    typedef std::map< std::string, int > ItemMap;
    BOOST_FOREACH (const ItemMap::value_type item, item_map) {
      const int32_t value(static_cast< int32_t >(item.second));
      const char *log(NULL);
      if (!data_->dxl_wb->itemWrite(data_->id, item.first.c_str(), value, &log)) {
        ROS_ERROR_STREAM("OperatingModeBase::writeItems(): Failed to set control table item '"
                         << item.first << "' of '" << data_->name
                         << "' (id: " << static_cast< int >(data_->id) << " to " << value << ": "
                         << (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
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
    const int32_t cmd_value(data_->dxl_wb->convertVelocity2Value(data_->id, cmd));
    const char *log(NULL);
    if (!data_->dxl_wb->itemWrite(data_->id, "Profile_Velocity", cmd_value, &log)) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::writeProfileVelocity(): Failed to set profile velocity of '"
          << data_->name << "' (id: " << static_cast< int >(data_->id) << ") to " << cmd
          << " (value: " << cmd_value
          << "): " << (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
      return false;
    }
    return true;
  }

  bool writeEffortCommand() {
    const float cmd(data_->eff_cmd / data_->torque_constant * 1000.0);
    // TODO: use new API once supported
    const int16_t cmd_value(data_->dxl_wb->convertCurrent2Value(
        /* data_->id, */ cmd));
    const char *log(NULL);
    if (!data_->dxl_wb->itemWrite(data_->id, "Goal_Current", cmd_value, &log)) {
      ROS_ERROR_STREAM("OperatingModeBase::writeEffortCommand(): Failed to set goal current of '"
                       << data_->name << "' (id: " << static_cast< int >(data_->id) << ") to "
                       << cmd << " (value: " << cmd_value
                       << "): " << (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
      return false;
    }
    return true;
  }

  bool writeAdditionalCommands() {
    typedef std::map< std::string, hie::ByteArray > CommandMap;
    BOOST_FOREACH (const CommandMap::value_type &cmd, data_->additional_cmds) {
      const int32_t value(cmd.second.to< int32_t >());
      const char *log(NULL);
      if (!data_->dxl_wb->itemWrite(data_->id, cmd.first.c_str(), value, &log)) {
        ROS_ERROR_STREAM("OperatingModeBase::writeAdditionalCommands(): Failed to set '"
                         << cmd.first << "' of '" << data_->name
                         << "' (id: " << static_cast< int >(data_->id) << " to " << value << ": "
                         << (log ? log : "No log from DynamixelWorkbench::itemWrite()"));
        return false;
      }
    }
    return true;
  }

  //
  // utility
  //

  static bool isNotNaN(const double a) { return !boost::math::isnan(a); }

  static bool areNotEqual(const double a, const double b) {
    // does !(|a - b| < EPS) instead of (|a - b| >= EPS) to return True when a and/or b is NaN
    return !(std::abs(a - b) < std::numeric_limits< double >::epsilon());
  }

protected:
  const std::string name_;
  const DynamixelActuatorDataPtr data_;
};

typedef boost::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef boost::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;
} // namespace layered_hardware_dynamixel

#endif