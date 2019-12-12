#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP

#include <cmath>
#include <limits>
#include <string>

#include <dynamixel_hardware/actuator_data.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <dynamixel/errors.hpp>
#include <dynamixel/operating_mode.hpp>
#include <dynamixel/protocols/protocol2.hpp>
#include <dynamixel/status_packet.hpp>

#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class ActuatorOperatingModeBase {
public:
  ActuatorOperatingModeBase(const std::string &name, const ActuatorDataPtr &data)
      : name_(name), data_(data) {}

  virtual ~ActuatorOperatingModeBase() {}

  std::string getName() const { return name_; }

  virtual void starting() {}

  virtual void read(const ros::Time &time, const ros::Duration &period) {}

  virtual void write(const ros::Time &time, const ros::Duration &period) {}

  virtual void stopping() {}

protected:
  //
  // read functions for chiled classes
  //

  void readPosition() {
    // send a request & receive a response
    dynamixel::StatusPacket< dp::Protocol2 > status;
    try {
      data_->device.send(data_->servo->get_present_position_angle());
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readPosition(): Failed to read dynamixel's position (id: "
          << data_->servo->id() << "): " << err.msg());
      return;
    }
    // validate the response
    if (!status.valid()) {
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readPosition(): Invalid status packet received "
                       "when reading dynamixel's position (id: "
                       << data_->servo->id() << ")");
      return;
    }
    // extract the position from the response
    try {
      data_->pos = data_->servo->parse_present_position_angle(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readPosition(): Failed to unpack dynamixel's position (id: "
          << data_->servo->id() << ")" << err.msg());
      return;
    }
  }

  void readVelocity() {
    // send a request & receive a response
    dynamixel::StatusPacket< dp::Protocol2 > status;
    try {
      data_->device.send(data_->servo->get_present_speed());
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readVelocity(): Failed to read dynamixel's velocity (id: "
          << data_->servo->id() << "): " << err.msg());
      return;
    }
    // validate the response
    if (!status.valid()) {
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readVelocity(): Invalid status packet received "
                       "when reading dynamixel's velocity (id: "
                       << data_->servo->id() << ")");
      return;
    }
    // extract the velocity from the response
    try {
      data_->vel = data_->servo->parse_joint_speed(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readVelocity(): Failed to unpack dynamixel's velocity (id: "
          << data_->servo->id() << ")" << err.msg());
      return;
    }
  }

  void readEffort() {
    // send a request & receive a response
    dynamixel::StatusPacket< dp::Protocol2 > status;
    try {
      data_->device.send(data_->servo->get_present_current());
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readEffort(): Failed to read dynamixel's current (id: "
          << data_->servo->id() << "): " << err.msg());
      return;
    }
    // validate the response
    if (!status.valid()) {
      ROS_ERROR_STREAM("ActuatorOperatingModeBase::readEffort(): Invalid status packet received "
                       "when reading dynamixel's current (id: "
                       << data_->servo->id() << ")");
      return;
    }
    // extract the current from the response
    try {
      data_->eff = static_cast< int16_t >(data_->servo->parse_present_current(status)) *
                   data_->torque_constant;
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::readEffort(): Failed to unpack dynamixel's current (id: "
          << data_->servo->id() << ")" << err.msg());
      return;
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

  void writeTorqueEnable(const bool value) {
    try {
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.send(data_->servo->set_torque_enable(value));
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeTorqueEnable(): Failed to write torque-enable ("
          << value << ") to " << data_->name);
      return;
    }
  }

  void writeOperatingMode(const dynamixel::OperatingMode &value) {
    try {
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.send(data_->servo->set_operating_mode(static_cast< long long >(value)));
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writePositionCommand(): Failed to write operating mode ("
          << dynamixel::mode2str(value) << ") to " << data_->name);
      return;
    }
  }

  void writePositionCommand() {
    try {
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.send(data_->servo->set_goal_position_angle(data_->pos_cmd));
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writePositionCommand(): Failed to write position command to "
          << data_->name);
      return;
    }
  }

  void writeVelocityCommand() {
    try {
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.send(data_->servo->set_moving_speed_angle(data_->vel_cmd));
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeVelocityCommand(): Failed to write velocity command to "
          << data_->name);
      return;
    }
  }

  void writeEffortCommand() {
    try {
      dynamixel::StatusPacket< dp::Protocol2 > status;
      data_->device.send(data_->servo->set_goal_current(static_cast< uint16_t >(
          static_cast< int16_t >(data_->eff_cmd / data_->torque_constant))));
      data_->device.recv(status);
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM(
          "ActuatorOperatingModeBase::writeEffortCommand(): Failed to write effort command to "
          << data_->name);
      return;
    }
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
  const ActuatorDataPtr data_;
};

typedef boost::shared_ptr< ActuatorOperatingModeBase > ActuatorOperatingModePtr;
typedef boost::shared_ptr< const ActuatorOperatingModeBase > ActuatorOperatingModeConstPtr;
} // namespace dynamixel_hardware

#endif