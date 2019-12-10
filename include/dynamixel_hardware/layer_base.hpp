#ifndef DYNAMIXEL_HARDWARE_LAYER_BASE_HPP
#define DYNAMIXEL_HARDWARE_LAYER_BASE_HPP

#include <list>
#include <string>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace dynamixel_hardware {

class LayerBase {
public:
  virtual bool init(hardware_interface::RobotHW &hw, ros::NodeHandle &param_nh,
                    const std::string &urdf_str) = 0;

  virtual void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                        const std::list< hardware_interface::ControllerInfo > &stop_list) = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;
};
} // namespace dynamixel_hardware

#endif