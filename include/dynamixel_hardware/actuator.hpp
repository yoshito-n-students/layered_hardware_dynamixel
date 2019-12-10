#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_HPP

#include <list>
#include <memory>
#include <string>

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/actuator_position_mode.hpp>
#include <dynamixel_hardware/actuator_reset_mode.hpp>
#include <dynamixel_hardware/actuator_torque_based_position_mode.hpp>
#include <dynamixel_hardware/actuator_torque_disable_mode.hpp>
#include <dynamixel_hardware/actuator_torque_mode.hpp>
#include <dynamixel_hardware/actuator_velocity_mode.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dynamixel/auto_detect.hpp> // for find_servo()
#include <dynamixel/controllers/usb2dynamixel.hpp>
#include <dynamixel/protocols/protocol2.hpp>
#include <dynamixel/servos/base_servo.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class Actuator {
public:
  Actuator(const std::string &name, dynamixel::controllers::Usb2Dynamixel &device)
      : name_(name), device_(device) {}

  virtual ~Actuator() {}

  bool init(hardware_interface::RobotHW &hw, ros::NodeHandle &param_nh) {
    { // register actuator states & commands to corresponding hardware interfaces
      const hardware_interface::ActuatorStateHandle state_handle(name_, &pos_, &vel_, &eff_);
      if (!registerActuatorTo< hardware_interface::ActuatorStateInterface >(hw, state_handle) ||
          !registerActuatorTo< hardware_interface::PositionActuatorInterface >(
              hw, hardware_interface::ActuatorHandle(state_handle, &pos_)) ||
          !registerActuatorTo< hardware_interface::VelocityActuatorInterface >(
              hw, hardware_interface::ActuatorHandle(state_handle, &vel_)) ||
          !registerActuatorTo< hardware_interface::EffortActuatorInterface >(
              hw, hardware_interface::ActuatorHandle(state_handle, &eff_))) {
        return false;
      }
    }

    { // dynamixel id from param
      int id_value;
      if (!param_nh.getParam("id", id_value)) {
        ROS_ERROR_STREAM("Actuator::init(): Failed to get param " << param_nh.resolveName("id"));
        return false;
      }
      id_ = id_value;
    }

    // find dynamixel actuator by id
    servo_ = dynamixel::find_servo< dynamixel::protocols::Protocol2 >(device_, id_);
    if (!servo_) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to find the actuator " << name_ << "(id: " << id_
                                                                        << ")");
      return false;
    }

    // TODO: init actuator based on params

    mode_ = dynamixel::OperatingMode::unknown;
  }

  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list) {
    // TODO: switch actuator's operating modes according to starting/stopping controllers
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    // TODO: read states from the actuator
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    // TODO: write commands to the actuator
  }

private:
  template < typename Interface, typename Handle >
  bool registerActuatorTo(hardware_interface::RobotHW &hw, const Handle &handle) {
    Interface *const iface(hw.get< Interface >());
    if (!iface) {
      ROS_ERROR("Actuator::registerActuator(): Failed to get a hardware interface");
      return false;
    }
    iface->registerHandle(handle);
    return true;
  }

  ActuatorOperatingModePtr makeOperatingMode(const std::string &mode_str) {
    if (mode_str == "monitor") {
      return boost::make_shared< ActuatorMonitorMode >(&pos_, &vel_, &eff_);
    } else if (mode_str == "torque_disable") {
      return boost::make_shared< ActuatorTorqueDisableMode >(&pos_, &vel_, &eff_);
    } else if (mode_str == "position") {
      return boost::make_shared< ActuatorPositionMode >(&pos_, &vel_, &eff_, &pos_cmd_);
    } else if (mode_str == "velocity") {
      return boost::make_shared< ActuatorVelocityMode >(&pos_, &vel_, &eff_, &vel_cmd_);
    } else if (mode_str == "torque") {
      return boost::make_shared< ActuatorTorqueMode >(&pos_, &vel_, &eff_, &eff_cmd_);
    } else if (mode_str == "torque_based_position") {
      return boost::make_shared< ActuatorTorqueBasedPositionMode >(&pos_, &vel_, &eff_, &pos_cmd_,
                                                                   &vel_cmd_, &eff_cmd_);
    } else if (mode_str == "reset") {
      return boost::make_shared< ActuatorResetMode >();
    }
    return ActuatorOperatingModePtr();
  }

private:
  const std::string name_;
  dynamixel::controllers::Usb2Dynamixel &device_;
  std::shared_ptr< dynamixel::servos::BaseServo< dynamixel::protocols::Protocol2 > > servo_;

  // params
  dynamixel::protocols::Protocol2::id_t id_;
  double torque_constant_;

  // states
  dynamixel::OperatingMode mode_;
  double pos_, vel_, eff_;

  // commands
  double pos_cmd_, vel_cmd_, eff_cmd_;
  double prev_pos_cmd_, prev_vel_cmd_, prev_eff_cmd_;
};

typedef boost::shared_ptr< Actuator > ActuatorPtr;
typedef boost::shared_ptr< const Actuator > ActuatorConstPtr;
} // namespace dynamixel_hardware

#endif