#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_HPP

#include <list>
#include <map>
#include <string>

#include <dynamixel_hardware/actuator_clear_multi_turn_mode.hpp>
#include <dynamixel_hardware/actuator_current_based_position_mode.hpp>
#include <dynamixel_hardware/actuator_current_mode.hpp>
#include <dynamixel_hardware/actuator_data.hpp>
#include <dynamixel_hardware/actuator_extended_position_mode.hpp>
#include <dynamixel_hardware/actuator_operating_mode_base.hpp>
#include <dynamixel_hardware/actuator_reboot_mode.hpp>
#include <dynamixel_hardware/actuator_torque_disable_mode.hpp>
#include <dynamixel_hardware/actuator_velocity_mode.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class Actuator {
public:
  Actuator() {}

  virtual ~Actuator() {
    // finalize the present mode
    if (present_mode_) {
      ROS_INFO_STREAM("Actuator::~Actuator(): Stopping operating mode "
                      << present_mode_->getName() << " for actuator " << data_->name);
      present_mode_->stopping();
      present_mode_ = ActuatorOperatingModePtr();
    }
  }

  bool init(const std::string &name, DynamixelWorkbench &dxl_wb, hi::RobotHW &hw,
            ros::NodeHandle &param_nh) {
    // dynamixel id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to get param " << param_nh.resolveName("id"));
      return false;
    }

    // find dynamixel actuator by id
    uint16_t model_number;
    if (!dxl_wb.ping(id, &model_number)) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to ping the actuator "
                       << name << "(id: " << static_cast< int >(id) << ")");
      return false;
    }

    // torque constant from param
    double torque_constant;
    if (!param_nh.getParam("torque_constant", torque_constant)) {
      ROS_ERROR_STREAM("Actuator::init(): Failed to get param "
                       << param_nh.resolveName("torque_constant"));
      return false;
    }

    // allocate data structure
    data_.reset(new ActuatorData(name, dxl_wb, id, torque_constant));

    // register actuator states & commands to corresponding hardware interfaces
    const hi::ActuatorStateHandle state_handle(data_->name, &data_->pos, &data_->vel, &data_->eff);
    if (!registerActuatorTo< hi::ActuatorStateInterface >(hw, state_handle) ||
        !registerActuatorTo< hi::PositionActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->pos_cmd)) ||
        !registerActuatorTo< hi::VelocityActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->vel_cmd)) ||
        !registerActuatorTo< hi::EffortActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->eff_cmd))) {
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
      std::map< std::string, int > item_map;
      param_nh.getParam(ros::names::append("item_map", mode_name.second), item_map);
      const ActuatorOperatingModePtr mode(makeOperatingMode(mode_name.second, item_map));
      if (!mode) {
        ROS_ERROR_STREAM("Actuator::init(): Failed to make operating mode "
                         << mode_name.second << " for " << data_->name);
        return false;
      }
      mode_map_[mode_name.first] = mode;
    }

    return true;
  }

  void doSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // stop actuator's operating mode according to stopping controller list
    if (present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const std::map< std::string, ActuatorOperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          ROS_INFO_STREAM("Actuator::doSwitch(): Stopping operating mode "
                          << present_mode_->getName() << " for actuator " << data_->name);
          present_mode_->stopping();
          present_mode_ = ActuatorOperatingModePtr();
          break;
        }
      }
    }

    // start actuator's operating modes according to starting controllers
    if (!present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
        const std::map< std::string, ActuatorOperatingModePtr >::const_iterator mode_to_start(
            mode_map_.find(starting_controller.name));
        if (mode_to_start != mode_map_.end() && mode_to_start->second) {
          ROS_INFO_STREAM("Actuator::doSwitch(): Starting operating mode "
                          << mode_to_start->second->getName() << " for actuator " << data_->name);
          present_mode_ = mode_to_start->second;
          present_mode_->starting();
          break;
        }
      }
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
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

  ActuatorOperatingModePtr makeOperatingMode(const std::string &mode_str,
                                             const std::map< std::string, int > &item_map) {
    if (mode_str == "clear_multi_turn") {
      return boost::make_shared< ActuatorClearMultiTurnMode >(data_);
    } else if (mode_str == "current") {
      return boost::make_shared< ActuatorCurrentMode >(data_, item_map);
    } else if (mode_str == "current_based_position") {
      return boost::make_shared< ActuatorCurrentBasedPositionMode >(data_, item_map);
    } else if (mode_str == "extended_position") {
      return boost::make_shared< ActuatorExtendedPositionMode >(data_, item_map);
    } else if (mode_str == "reboot") {
      return boost::make_shared< ActuatorRebootMode >(data_);
    } else if (mode_str == "torque_disable") {
      return boost::make_shared< ActuatorTorqueDisableMode >(data_);
    } else if (mode_str == "velocity") {
      return boost::make_shared< ActuatorVelocityMode >(data_, item_map);
    }
    ROS_ERROR_STREAM("Actuator::makeOperatingMode(): Unknown operating mode name " << mode_str);
    return ActuatorOperatingModePtr();
  }

private:
  ActuatorDataPtr data_;

  std::map< std::string, ActuatorOperatingModePtr > mode_map_;
  ActuatorOperatingModePtr present_mode_;
};

typedef boost::shared_ptr< Actuator > ActuatorPtr;
typedef boost::shared_ptr< const Actuator > ActuatorConstPtr;
} // namespace dynamixel_hardware

#endif