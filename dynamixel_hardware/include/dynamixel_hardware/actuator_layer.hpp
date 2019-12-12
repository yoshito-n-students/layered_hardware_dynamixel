#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_LAYER_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_LAYER_HPP

#include <list>
#include <string>
#include <vector>

#include <dynamixel_hardware/actuator.hpp>
#include <dynamixel_hardware/common_namespaces.hpp>
#include <dynamixel_hardware/layer_base.hpp>
#include <dynamixel_hardware/usb2dynamixel_serial.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <dynamixel/controllers/usb2dynamixel.hpp>
#include <dynamixel/errors.hpp>
#include <dynamixel/misc.hpp> // for get_baudrate()

#include <boost/foreach.hpp>

namespace dynamixel_hardware {

class ActuatorLayer : public LayerBase {
public:
  virtual bool init(hi::RobotHW &hw, ros::NodeHandle &param_nh, const std::string &urdf_str) {
    // register actuator interfaces to the hardware so that other layers can find the interfaces
    hw.registerInterface(&state_iface_);
    hw.registerInterface(&pos_iface_);
    hw.registerInterface(&vel_iface_);
    hw.registerInterface(&eff_iface_);

    // open USB serial device
    try {
      device_.open_serial(param_nh.param< std::string >("serial_interface", "/dev/ttyUSB0"),
                          param_nh.param("baudrate", 115200));
      device_.set_recv_timeout(param_nh.param("read_timeout", 0.05));
    } catch (const de::Error &err) {
      ROS_ERROR_STREAM("ActuatorLayer::init(): Failed to open Usb2Dynamixel: " << err.msg());
      return false;
    }

    // load actuator names from param "actuators"
    XmlRpc::XmlRpcValue ators_param;
    if (!param_nh.getParam("actuators", ators_param)) {
      ROS_ERROR_STREAM("ActuatorLayer::init(): Failed to get param "
                       << param_nh.resolveName("actuators"));
      return false;
    }
    if (ators_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("ActuatorLayer::init(): Param " << param_nh.resolveName("actuators")
                                                       << " must be a struct");
      return false;
    }

    // init actuators with param "actuators/<actuator_name>"
    BOOST_FOREACH (const XmlRpc::XmlRpcValue::ValueStruct::value_type &ator_param, ators_param) {
      ActuatorPtr ator(new Actuator());
      ros::NodeHandle ator_param_nh(param_nh, ros::names::append("actuators", ator_param.first));
      if (!ator->init(ator_param.first, device_, hw, ator_param_nh)) {
        return false;
      }
      actuators_.push_back(ator);
    }

    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // notify controller switching to all actuators
    BOOST_FOREACH (const ActuatorPtr &ator, actuators_) { ator->doSwitch(start_list, stop_list); }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read from all actuators
    BOOST_FOREACH (const ActuatorPtr &ator, actuators_) { ator->read(time, period); }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all actuators
    BOOST_FOREACH (const ActuatorPtr &ator, actuators_) { ator->write(time, period); }
  }

private:
  hi::ActuatorStateInterface state_iface_;
  hi::PositionActuatorInterface pos_iface_;
  hi::VelocityActuatorInterface vel_iface_;
  hi::EffortActuatorInterface eff_iface_;

  // TODO: replace Usb2Dynamixel to Usb2DynamixelSerial
  dc::Usb2Dynamixel device_;
  std::vector< ActuatorPtr > actuators_;
};
} // namespace dynamixel_hardware

#endif