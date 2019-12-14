#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_DATA_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_DATA_HPP

#include <cstdint>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

struct ActuatorData {
  ActuatorData(const std::string &_name, DynamixelWorkbench &_dxl_wb, const uint8_t _id,
               const double _torque_constant)
      : name(_name), dxl_wb(_dxl_wb), id(_id), torque_constant(_torque_constant), pos(0.), vel(0.),
        eff(0.), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {}

  // handles
  const std::string name;
  DynamixelWorkbench &dxl_wb;
  const uint8_t id;

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