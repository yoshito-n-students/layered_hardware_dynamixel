#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_DATA_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_DATA_HPP

#include <map>
#include <string>
#include <vector>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <hardware_interface_extensions/byte_array_interface.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_dynamixel {

struct DynamixelActuatorData {
  DynamixelActuatorData(const std::string &_name, DynamixelWorkbench *const _dxl_wb,
                        const uint8_t _id, const double _torque_constant,
                        const std::vector< std::string > &additional_state_names,
                        const std::vector< std::string > &additional_cmd_names)
      : name(_name), dxl_wb(_dxl_wb), id(_id), torque_constant(_torque_constant), pos(0.), vel(0.),
        eff(0.), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {
    // TODO: this sorts names and breaks the original order.
    //       use std::vector< std::pair<> > instead of std::map<> .
    BOOST_FOREACH (const std::string &name, additional_state_names) {
      additional_states[name] = hie::ByteArray::from< int32_t >(0);
    }
    BOOST_FOREACH (const std::string &name, additional_cmd_names) {
      additional_cmds[name] = hie::ByteArray::from< int32_t >(0);
    }
  }

  // handles
  const std::string name;
  DynamixelWorkbench *const dxl_wb;
  const uint8_t id;

  // params
  const double torque_constant;

  // states
  double pos, vel, eff;
  std::map< std::string, hie::ByteArray > additional_states;

  // commands
  double pos_cmd, vel_cmd, eff_cmd;
  std::map< std::string, hie::ByteArray > additional_cmds;
};

typedef boost::shared_ptr< DynamixelActuatorData > DynamixelActuatorDataPtr;
typedef boost::shared_ptr< const DynamixelActuatorData > DynamixelActuatorDataConstPtr;

} // namespace layered_hardware_dynamixel

#endif