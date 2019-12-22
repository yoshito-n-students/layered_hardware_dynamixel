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

  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  //
  // read functions for chiled classes
  //

  void readPosition() {
    float rad;
    if (data_->dxl_wb.getRadian(data_->id, &rad)) {
      data_->pos = rad;
    } else {
      ROS_ERROR_STREAM("OperatingModeBase::readPosition(): Failed to read position from "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void readVelocity() {
    float radps;
    if (data_->dxl_wb.getVelocity(data_->id, &radps)) {
      data_->vel = radps;
    } else {
      ROS_ERROR_STREAM("OperatingModeBase::readVelocity(): Failed to read velocity from "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void readEffort() {
    int32_t value;
    // TODO:
    if (data_->dxl_wb.itemRead(data_->id, "Present_Current", &value)) {
      // TODO: use the latest API
      data_->eff =
          data_->dxl_wb.convertValue2Current(/* data_->id,*/ value) * data_->torque_constant;
    } else {
      ROS_ERROR_STREAM("OperatingModeBase::readEffort(): Failed to read current from "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void readState() {
    readPosition();
    readVelocity();
    readEffort();
  }

  //
  // write functions for child classes
  //

  void setOperatingModeAndTorqueOn(bool (DynamixelWorkbench::*const set_func)(uint8_t,
                                                                              const char **)) {
    if (!(data_->dxl_wb.*set_func)(data_->id, NULL)) {
      ROS_ERROR_STREAM("OperatingModeBase::setOperatingModeAndTorqueOn(): Failed to set "
                       "operating mode of "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
      return;
    }
    if (!data_->dxl_wb.torqueOn(data_->id)) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::setOperatingModeAndTorqueOn(): Failed to enable torque of "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
      return;
    }
  }

  void writeItems(const std::map< std::string, int > &item_map) {
    typedef std::map< std::string, int > ItemMap;
    BOOST_FOREACH (const ItemMap::value_type item, item_map) {
      if (!data_->dxl_wb.itemWrite(data_->id, item.first.c_str(),
                                   static_cast< int32_t >(item.second))) {
        ROS_ERROR_STREAM("OperatingModeBase::writeItems(): Failed to write control item "
                         << item.first << " of " << data_->name << " to " << item.second);
      }
    }
  }

  void torqueOff() {
    if (!data_->dxl_wb.torqueOff(data_->id)) {
      ROS_ERROR_STREAM("OperatingModeBase::torqueOff(): Failed to disable torque of "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writePositionCommand() {
    if (!data_->dxl_wb.goalPosition(data_->id, static_cast< float >(data_->pos_cmd))) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::writePositionCommand(): Failed to write position command to "
          << data_->name << " (command: " << data_->pos_cmd
          << ", id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeVelocityCommand() {
    if (!data_->dxl_wb.goalVelocity(data_->id, static_cast< float >(data_->vel_cmd))) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::writeVelocityCommand(): Failed to write velocity command to "
          << data_->name << " (command: " << data_->vel_cmd
          << ", id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeProfileVelocity() {
    const double prof_vel_cmd(std::abs(data_->vel_cmd));
    if (!data_->dxl_wb.itemWrite(data_->id, "Profile_Velocity",
                                 data_->dxl_wb.convertVelocity2Value(data_->id, prof_vel_cmd))) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::writeProfileVelocity(): Failed to write profile velocity to "
          << data_->name << " (command: " << prof_vel_cmd
          << ", id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeEffortCommand() {
    const double cur_cmd(data_->eff_cmd / data_->torque_constant);
    if (!data_->dxl_wb.itemWrite(data_->id, "Goal_Current",
                                 // TODO: use the latest API
                                 data_->dxl_wb.convertCurrent2Value(
                                     /* data_->id, */ cur_cmd))) {
      ROS_ERROR_STREAM(
          "OperatingModeBase::writeEffortCommand(): Failed to write current command to "
          << data_->name << " (command: " << cur_cmd << ", id: " << static_cast< int >(data_->id)
          << ")");
    }
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