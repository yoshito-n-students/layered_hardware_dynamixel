#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_DATA_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_DATA_HPP

#include <memory>
#include <string>

#include <dynamixel_hardware/common_namespaces.hpp>

#include <dynamixel/controllers/usb2dynamixel.hpp>
#include <dynamixel/protocols/protocol2.hpp>
#include <dynamixel/servos/base_servo.hpp>

#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

struct ActuatorData {
  ActuatorData(const std::string &_name, dc::Usb2Dynamixel &_device,
               const std::shared_ptr< ds::BaseServo< dp::Protocol2 > > &_servo,
               const double _torque_constant)
      : name(_name), device(_device), servo(_servo), torque_constant(_torque_constant), pos(0.),
        vel(0.), eff(0.), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {}

  // handles
  const std::string name;
  // TODO: replace Usb2Dynamixel to Usb2DynamixelSerial
  dc::Usb2Dynamixel &device;
  const std::shared_ptr< ds::BaseServo< dp::Protocol2 > > servo;

  // params
  const double torque_constant;

  // states
  double pos, vel, eff;

  // commands
  double pos_cmd, vel_cmd, eff_cmd;
};

typedef boost::shared_ptr< ActuatorData > ActuatorDataPtr;
typedef boost::shared_ptr< const ActuatorData > ActuatorDataConstPtr;

} // namespace dynamixel_hardware

#endif