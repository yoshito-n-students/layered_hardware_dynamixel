#ifndef DYNAMIXEL_HARDWARE_TRANSMISSION_LAYER_HPP
#define DYNAMIXEL_HARDWARE_TRANSMISSION_LAYER_HPP

#include <algorithm>
#include <list>
#include <string>
#include <vector>

#include <dynamixel_hardware/layer_base.hpp>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_parser.h>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

namespace dynamixel_hardware {

class TransmissionLayer : public LayerBase {
public:
  virtual bool init(hardware_interface::RobotHW &hw, ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    // extract transmission informations from URDF
    std::vector< transmission_interface::TransmissionInfo > infos;
    if (!transmission_interface::TransmissionParser::parse(urdf_str, infos)) {
      ROS_ERROR("TransmissionLayer::init(): Failed to parse transmissions from URDF");
      return false;
    }

    // get actuator names already registered in the hardware
    const hardware_interface::ActuatorStateInterface *const ator_iface(
        hw.get< hardware_interface::ActuatorStateInterface >());
    if (!ator_iface) {
      ROS_ERROR("TransmissionLayer::init(): No actuator registered");
      return false;
    }
    const std::vector< std::string > hw_ator_names(ator_iface->getNames());

    // load all transmissions for actuators in the hardware
    iface_loader_.reset(
        new transmission_interface::TransmissionInterfaceLoader(&hw, &transmissions_));
    BOOST_FOREACH (const transmission_interface::TransmissionInfo &info, infos) {
      // confirm the transmission is for some of actuators in the hardware
      bool has_non_hw_ator(false);
      BOOST_FOREACH (const transmission_interface::ActuatorInfo &ator_info, info.actuators_) {
        if (std::find(hw_ator_names.begin(), hw_ator_names.end(), ator_info.name_) ==
            hw_ator_names.end()) {
          has_non_hw_ator = true;
          break;
        }
      }
      if (has_non_hw_ator) {
        continue;
      }
      // load the transmission
      if (!iface_loader_->load(info)) {
        ROS_ERROR_STREAM("TransmissionLayer::init(): Failed to load the transmission "
                         << info.name_);
        return false;
      }
    }

    return true;
  }

  virtual void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                        const std::list< hardware_interface::ControllerInfo > &stop_list) {
    // nothing to do
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read joint state from actuator state
    propagate< transmission_interface::ActuatorToJointStateInterface >();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write joint commands to actuator commands
    propagate< transmission_interface::JointToActuatorPositionInterface >();
    propagate< transmission_interface::JointToActuatorVelocityInterface >();
    propagate< transmission_interface::JointToActuatorEffortInterface >();
  }

private:
  template < typename Interface > void propagate() {
    Interface *const iface(transmissions_.get< Interface >());
    if (iface) {
      iface->propagate();
    }
  }

private:
  transmission_interface::RobotTransmissions transmissions_;
  boost::scoped_ptr< transmission_interface::TransmissionInterfaceLoader > iface_loader_;
};
} // namespace dynamixel_hardware

#endif