#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP

#include <list>
#include <map>
#include <string>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware_dynamixel/clear_multi_turn_mode.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/current_based_position_mode.hpp>
#include <layered_hardware_dynamixel/current_mode.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_data.hpp>
#include <layered_hardware_dynamixel/extended_position_mode.hpp>
#include <layered_hardware_dynamixel/operating_mode_base.hpp>
#include <layered_hardware_dynamixel/reboot_mode.hpp>
#include <layered_hardware_dynamixel/torque_disable_mode.hpp>
#include <layered_hardware_dynamixel/velocity_mode.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_dynamixel {

class DynamixelActuator {
public:
  DynamixelActuator() {}

  virtual ~DynamixelActuator() {
    // finalize the present mode
    if (present_mode_) {
      ROS_INFO_STREAM("DynamixelActuator::~DynamixelActuator(): Stopping operating mode '"
                      << present_mode_->getName() << "' for actuator '" << data_->name << "'");
      present_mode_->stopping();
      present_mode_ = OperatingModePtr();
    }
  }

  bool init(const std::string &name, DynamixelWorkbench &dxl_wb, hi::RobotHW &hw,
            ros::NodeHandle &param_nh) {
    // dynamixel id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("id") << "'");
      return false;
    }

    // find dynamixel actuator by id
    uint16_t model_number;
    if (!dxl_wb.ping(id, &model_number)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to ping the actuator '"
                       << name << "' (id: " << static_cast< int >(id) << ")");
      return false;
    }

    // torque constant from param
    double torque_constant;
    if (!param_nh.getParam("torque_constant", torque_constant)) {
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("torque_constant") << "'");
      return false;
    }

    // allocate data structure
    data_.reset(new DynamixelActuatorData(name, dxl_wb, id, torque_constant));

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
      ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to get param '"
                       << param_nh.resolveName("operating_mode_map") << "'");
      return false;
    }
    BOOST_FOREACH (const ModeNameMap::value_type &mode_name, mode_name_map) {
      std::map< std::string, int > item_map;
      param_nh.getParam(ros::names::append("item_map", mode_name.second), item_map);
      const OperatingModePtr mode(makeOperatingMode(mode_name.second, item_map));
      if (!mode) {
        ROS_ERROR_STREAM("DynamixelActuator::init(): Failed to make operating mode '"
                         << mode_name.second << "' for the actuator '" << data_->name << "'");
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
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          ROS_INFO_STREAM("DynamixelActuator::doSwitch(): Stopping operating mode '"
                          << present_mode_->getName() << "' for the actuator '" << data_->name
                          << "'");
          present_mode_->stopping();
          present_mode_ = OperatingModePtr();
          break;
        }
      }
    }

    // start actuator's operating modes according to starting controllers
    if (!present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_start(
            mode_map_.find(starting_controller.name));
        if (mode_to_start != mode_map_.end() && mode_to_start->second) {
          ROS_INFO_STREAM("DynamixelActuator::doSwitch(): Starting operating mode '"
                          << mode_to_start->second->getName() << "' for the actuator '"
                          << data_->name << "'");
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
      ROS_ERROR("DynamixelActuator::registerActuatorTo(): Failed to get a hardware interface");
      return false;
    }
    iface->registerHandle(handle);
    return true;
  }

  OperatingModePtr makeOperatingMode(const std::string &mode_str,
                                     const std::map< std::string, int > &item_map) {
    if (mode_str == "clear_multi_turn") {
      return boost::make_shared< ClearMultiTurnMode >(data_);
    } else if (mode_str == "current") {
      return boost::make_shared< CurrentMode >(data_, item_map);
    } else if (mode_str == "current_based_position") {
      return boost::make_shared< CurrentBasedPositionMode >(data_, item_map);
    } else if (mode_str == "extended_position") {
      return boost::make_shared< ExtendedPositionMode >(data_, item_map);
    } else if (mode_str == "reboot") {
      return boost::make_shared< RebootMode >(data_);
    } else if (mode_str == "torque_disable") {
      return boost::make_shared< TorqueDisableMode >(data_);
    } else if (mode_str == "velocity") {
      return boost::make_shared< VelocityMode >(data_, item_map);
    }
    ROS_ERROR_STREAM("DynamixelActuator::makeOperatingMode(): Unknown operating mode name '"
                     << mode_str << "'");
    return OperatingModePtr();
  }

private:
  DynamixelActuatorDataPtr data_;

  std::map< std::string, OperatingModePtr > mode_map_;
  OperatingModePtr present_mode_;
};

typedef boost::shared_ptr< DynamixelActuator > DynamixelActuatorPtr;
typedef boost::shared_ptr< const DynamixelActuator > DynamixelActuatorConstPtr;
} // namespace layered_hardware_dynamixel

#endif