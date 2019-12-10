#ifndef DYNAMIXEL_HARDWARE_DYNAMIXEL_HARDWARE_HPP
#define DYNAMIXEL_HARDWARE_DYNAMIXEL_HARDWARE_HPP

#include <list>
#include <string>

#include <dynamixel_hardware/actuator_layer.hpp>
#include <dynamixel_hardware/joint_limits_layer.hpp>
#include <dynamixel_hardware/transmission_layer.hpp>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class DynamixelHardware : public hardware_interface::RobotHW {
public:
  bool init(ros::NodeHandle &param_nh) {
    // get URDF description from param
    std::string urdf_str;
    if (!param_nh.getParam("robot_description", urdf_str) &&
        !ros::param::get("robot_description", urdf_str)) {
      ROS_ERROR("Failed to get URDF description from param");
      return false;
    }

    // init layers from bottom to upper
    return actuator_layer_.init(*this, param_nh, urdf_str) &&
           transmission_layer_.init(*this, param_nh, urdf_str) &&
           joint_limits_layer_.init(*this, param_nh, urdf_str);
  }

  virtual void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                        const std::list< hardware_interface::ControllerInfo > &stop_list) {
    // do something required on switching controllers
    joint_limits_layer_.doSwitch(start_list, stop_list);
    transmission_layer_.doSwitch(start_list, stop_list);
    actuator_layer_.doSwitch(start_list, stop_list);
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read states from actuators
    actuator_layer_.read(time, period);
    transmission_layer_.read(time, period);
    joint_limits_layer_.read(time, period);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write commands to actuators
    joint_limits_layer_.write(time, period);
    transmission_layer_.write(time, period);
    actuator_layer_.write(time, period);
  }

private:
  ActuatorLayer actuator_layer_;
  TransmissionLayer transmission_layer_;
  JointLimitsLayer joint_limits_layer_;
};
} // namespace dynamixel_hardware

#endif