#ifndef DYNAMIXEL_HARDWARE_LAYER_BASE_HPP
#define DYNAMIXEL_HARDWARE_LAYER_BASE_HPP

#include <list>
#include <string>

#include <dynamixel_hardware/common_namespaces.hpp>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class LayerBase {
public:
  virtual bool init(hi::RobotHW &hw, ros::NodeHandle &param_nh, const std::string &urdf_str) = 0;

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;
};
} // namespace dynamixel_hardware

#endif