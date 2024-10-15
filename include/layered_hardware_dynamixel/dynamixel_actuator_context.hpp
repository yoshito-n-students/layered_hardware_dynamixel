#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_CONTEXT_HPP

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <layered_hardware_dynamixel/common_namespaces.hpp>

namespace layered_hardware_dynamixel {

struct DynamixelActuatorContext {
  // handles
  const std::string name;
  const std::shared_ptr<DynamixelWorkbench> dxl_wb;
  const std::uint8_t id;

  // params
  const double torque_constant;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

// utility functions

static inline std::string
get_display_name(const std::shared_ptr<DynamixelActuatorContext> &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context->name << "\" actuator (id: " << static_cast<int>(context->id) << ")";
  return disp_name.str();
}

// convenient to check change on xxx_cmd by comparing with previous value
static inline bool bitwise_equal(const double a, const double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

} // namespace layered_hardware_dynamixel

#endif