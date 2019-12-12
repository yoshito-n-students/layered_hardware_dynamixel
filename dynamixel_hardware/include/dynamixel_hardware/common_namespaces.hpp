#ifndef DYNAMIXEL_HARDWARE_COMMON_NAMESPACES_HPP
#define DYNAMIXEL_HARDWARE_COMMON_NAMESPACES_HPP

namespace dynamixel {
namespace controllers {}
namespace errors {}
namespace instructions {}
namespace protocols {}
namespace servos {}
} // namespace dynamixel

namespace hardware_interface {}

namespace joint_limits_interface {}

namespace transmission_interface {}

namespace dynamixel_hardware {
namespace dc = dynamixel::controllers;
namespace de = dynamixel::errors;
namespace di = dynamixel::instructions;
namespace dp = dynamixel::protocols;
namespace ds = dynamixel::servos;
namespace hi = hardware_interface;
namespace jli = joint_limits_interface;
namespace ti = transmission_interface;
} // namespace dynamixel_hardware

#endif