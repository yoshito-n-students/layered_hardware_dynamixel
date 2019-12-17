#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <dynamixel_hardware/actuator_data.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/math/special_functions/fpclassify.hpp> // for isnan()
#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class ActuatorOperatingModeBase {
public:
  ActuatorOperatingModeBase(const std::string &name, const ActuatorDataPtr &data)
      : name_(name), data_(data) {}

  virtual ~ActuatorOperatingModeBase() {}

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
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readPosition(): Failed to read position from "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void readVelocity() {
    float radps;
    if (data_->dxl_wb.getVelocity(data_->id, &radps)) {
      data_->vel = radps;
    } else {
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readVelocity(): Failed to read velocity from "
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
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readEffort(): Failed to read current from "
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
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::setOperatingModeAndTorqueOn(): Failed to set "
                       "operating mode of "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
      return;
    }
    if (!data_->dxl_wb.torqueOn(data_->id)) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::setOperatingModeAndTorqueOn(): Failed to enable torque of "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
      return;
    }
  }

  void torqueOff() {
    if (!data_->dxl_wb.torqueOff(data_->id)) {
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::torqueOff(): Failed to disable torque of "
                       << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writePositionCommand() {
    if (!data_->dxl_wb.goalPosition(data_->id, static_cast< float >(data_->pos_cmd))) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writePositionCommand(): Failed to write position command to "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeVelocityCommand() {
    if (!data_->dxl_wb.goalVelocity(data_->id, static_cast< float >(data_->vel_cmd))) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeVelocityCommand(): Failed to write velocity command to "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeProfileVelocity() {
    if (!data_->dxl_wb.itemWrite(data_->id, "Profile_Velocity",
                                 data_->dxl_wb.convertVelocity2Value(data_->id, data_->vel_cmd))) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeProfileVelocity(): Failed to write profile velocity to "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
    }
  }

  void writeEffortCommand() {
    if (!data_->dxl_wb.itemWrite(data_->id, "Goal_Current",
                                 // TODO: use the latest API
                                 data_->dxl_wb.convertCurrent2Value(
                                     /* data_->id, */ data_->eff_cmd / data_->torque_constant))) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeEffortCommand(): Failed to write current command to "
          << data_->name << " (id: " << static_cast< int >(data_->id) << ")");
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
  const ActuatorDataPtr data_;
};

typedef boost::shared_ptr< ActuatorOperatingModeBase > ActuatorOperatingModePtr;
typedef boost::shared_ptr< const ActuatorOperatingModeBase > ActuatorOperatingModeConstPtr;
} // namespace dynamixel_hardware

#endif