#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_HPP

#include <list>
#include <map>
#include <memory>
#include <string>

#include <dynamixel_hardware/actuator_monitor_mode.hpp>
#include <dynamixel_hardware/actuator_null_mode.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/actuator_position_mode.hpp>
#include <dynamixel_hardware/actuator_reset_mode.hpp>
#include <dynamixel_hardware/actuator_torque_based_position_mode.hpp>
#include <dynamixel_hardware/actuator_torque_disable_mode.hpp>
#include <dynamixel_hardware/actuator_torque_mode.hpp>
#include <dynamixel_hardware/actuator_velocity_mode.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
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

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class Actuator {
public:
  Actuator(const std::string &name, dc::Usb2Dynamixel &device) : name_(name), device_(device) {}

  virtual ~Actuator() {}

  bool init(hi::RobotHW &hw, ros::NodeHandle &param_nh) {
    // register actuator states & commands to corresponding hardware interfaces
    const hi::ActuatorStateHandle state_handle(name_, &pos_, &vel_, &eff_);
    if (!registerActuatorTo< hi::ActuatorStateInterface >(hw, state_handle) ||
        !registerActuatorTo< hi::PositionActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &pos_)) ||
        !registerActuatorTo< hi::VelocityActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &vel_)) ||
        !registerActuatorTo< hi::EffortActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &eff_))) {
      return false;
    }

    // dynamixel id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to get param " << param_nh.resolveName("id"));
      return false;
    }

    // find dynamixel actuator by id
    servo_ = dynamixel::find_servo< dp::Protocol2 >(device_, id);
    if (!servo_) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to find the actuator " << name_ << "(id: " << id
                                                                        << ")");
      return false;
    }

    // make operating mode map from ros-controller name to dynamixel's operating mode
    typedef std::map< std::string, std::string > ModeNameMap;
    ModeNameMap mode_name_map;
    if (!param_nh.getParam("operating_mode_map", mode_name_map)) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to get param "
                       << param_nh.resolveName("operating_mode_map"));
      return false;
    }
    BOOST_FOREACH (const ModeNameMap::value_type &mode_name, mode_name_map) {
      const ActuatorOperatingModePtr mode(makeOperatingMode(mode_name.second));
      if (!mode) {
        ROS_ERROR_STREAM("Actuator::init(): Failed to make operating mode " << mode_name.second
                                                                            << " for " << name_);
        return false;
      }
      mode_map_[mode_name.first] = mode;
    }

    // TODO: init actuator based on params

    present_mode_ = boost::make_shared< ActuatorNullMode >();
  }

  void doSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // switch actuator's operating modes according to starting controllers
    BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
      // find mode to switch
      const std::map< std::string, ActuatorOperatingModePtr >::const_iterator mode_to_switch(
          mode_map_.find(starting_controller.name));
      if (mode_to_switch == mode_map_.end()) {
        continue;
      }
      // switch modes
      present_mode_->stopping();
      present_mode_ = mode_to_switch->second;
      present_mode_->starting();
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    present_mode_->read(time, period);
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    present_mode_->write(time, period);
  }

private:
  template < typename Interface, typename Handle >
  bool registerActuatorTo(hi::RobotHW &hw, const Handle &handle) {
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
    } else if (mode_str == "null") {
      return boost::make_shared< ActuatorNullMode >();
    }
    ROS_ERROR_STREAM("Actuator::makeOperatingMode(): Unknown operating mode name " << mode_str);
    return ActuatorOperatingModePtr();
  }

private:
  const std::string name_;
  dc::Usb2Dynamixel &device_;
  std::shared_ptr< ds::BaseServo< dp::Protocol2 > > servo_;
  std::map< std::string, ActuatorOperatingModePtr > mode_map_;
  ActuatorOperatingModePtr present_mode_;

  // params
  double torque_constant_;

  // states
  double pos_, vel_, eff_;

  // commands
  double pos_cmd_, vel_cmd_, eff_cmd_;
  double prev_pos_cmd_, prev_vel_cmd_, prev_eff_cmd_;
};

typedef boost::shared_ptr< Actuator > ActuatorPtr;
typedef boost::shared_ptr< const Actuator > ActuatorConstPtr;
} // namespace dynamixel_hardware

#endif