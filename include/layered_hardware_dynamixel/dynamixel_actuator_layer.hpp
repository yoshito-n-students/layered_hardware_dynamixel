#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_LAYER_HPP

#include <list>
#include <string>
#include <vector>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/controller_set.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/foreach.hpp>

namespace layered_hardware_dynamixel {

class DynamixelActuatorLayer : public lh::LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    // make actuator interfaces registered to the hardware
    // so that other layers can find the interfaces
    makeRegistered< hi::ActuatorStateInterface >(hw);
    makeRegistered< hi::PositionActuatorInterface >(hw);
    makeRegistered< hi::VelocityActuatorInterface >(hw);
    makeRegistered< hi::EffortActuatorInterface >(hw);

    // open USB serial device
    if (!dxl_wb_.init(param< std::string >(param_nh, "serial_interface", "/dev/ttyUSB0").c_str(),
                      param(param_nh, "baudrate", 115200))) {
      ROS_ERROR_STREAM("DynamixelActuatorLayer::init(): Failed to open DynamixelWorkbench");
      return false;
    }

    // load actuator names from param "actuators"
    XmlRpc::XmlRpcValue ators_param;
    if (!param_nh.getParam("actuators", ators_param)) {
      ROS_ERROR_STREAM("DynamixelActuatorLayer::init(): Failed to get param '"
                       << param_nh.resolveName("actuators") << "'");
      return false;
    }
    if (ators_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("DynamixelActuatorLayer::init(): Param '"
                       << param_nh.resolveName("actuators") << "' must be a struct");
      return false;
    }

    // init actuators with param "actuators/<actuator_name>"
    // (could not use BOOST_FOREACH here to avoid a bug in the library in Kinetic)
    for (XmlRpc::XmlRpcValue::iterator ator_param = ators_param.begin();
         ator_param != ators_param.end(); ++ator_param) {
      DynamixelActuatorPtr ator(new DynamixelActuator());
      ros::NodeHandle ator_param_nh(param_nh, ros::names::append("actuators", ator_param->first));
      if (!ator->init(ator_param->first, &dxl_wb_, hw, ator_param_nh)) {
        return false;
      }
      ROS_INFO_STREAM("DynamixelActuatorLayer::init(): Initialized the actuator '"
                      << ator_param->first << "'");
      actuators_.push_back(ator);
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // dry-update of the list of running controllers
    const ControllerSet updated_list(controllers_.updated(start_list, stop_list));

    // ask to all actuators if controller switching is possible
    BOOST_FOREACH (const DynamixelActuatorPtr &ator, actuators_) {
      if (!ator->prepareSwitch(updated_list)) {
        return false;
      }
    }

    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // update the list of running controllers
    controllers_.update(start_list, stop_list);

    // notify controller switching to all actuators
    BOOST_FOREACH (const DynamixelActuatorPtr &ator, actuators_) {
      ator->doSwitch(controllers_);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read from all actuators
    BOOST_FOREACH (const DynamixelActuatorPtr &ator, actuators_) { ator->read(time, period); }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all actuators
    BOOST_FOREACH (const DynamixelActuatorPtr &ator, actuators_) { ator->write(time, period); }
  }

private:
  // make an hardware interface registered. the interface must be in the static memory space
  // to allow access from outside of this plugin.
  template < typename Interface > static void makeRegistered(hi::RobotHW *const hw) {
    if (!hw->get< Interface >()) {
      static Interface iface;
      hw->registerInterface(&iface);
    }
  }

private:
  DynamixelWorkbench dxl_wb_;
  ControllerSet controllers_;
  std::vector< DynamixelActuatorPtr > actuators_;
};
} // namespace layered_hardware_dynamixel

#endif