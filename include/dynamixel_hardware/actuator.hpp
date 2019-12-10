#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_HPP

#include <list>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dynamixel/controllers/usb2dynamixel.hpp>
#include <dynamixel/protocols/protocol2.hpp>

#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class Actuator {
public:
  Actuator(dynamixel::controllers::Usb2Dynamixel &device) : device_(device) {}

  virtual ~Actuator() {}

  bool init(hardware_interface::RobotHW &hw, ros::NodeHandle &param_nh) {}

  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list) {}

  void read(const ros::Time &time, const ros::Duration &period) {}

  void write(const ros::Time &time, const ros::Duration &period) {}

private:
  dynamixel::controllers::Usb2Dynamixel &device_;

  // params
  dynamixel::protocols::Protocol2::id_t id_;
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