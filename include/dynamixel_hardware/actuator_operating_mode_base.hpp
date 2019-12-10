#ifndef DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP
#define DYNAMIXEL_HARDWARE_ACTUATOR_OPERATING_MODE_BASE_HPP

#include <ros/duration.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace dynamixel_hardware {

class ActuatorOperatingModeBase {
public:
  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;
};

typedef boost::shared_ptr< ActuatorOperatingModeBase > ActuatorOperatingModePtr;
typedef boost::shared_ptr< const ActuatorOperatingModeBase > ActuatorOperatingModeConstPtr;
} // namespace dynamixel_hardware

#endif