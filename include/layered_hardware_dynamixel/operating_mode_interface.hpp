#ifndef LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_INTERFACE_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_OPERATING_MODE_INTERFACE_HPP

#include <memory>
#include <string>

#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_dynamixel {

class OperatingModeInterface {
public:
  OperatingModeInterface(const std::string &name,
                         const std::shared_ptr<DynamixelActuatorContext> &context)
      : name_(name), context_(context) {}

  virtual ~OperatingModeInterface() {}

  std::string get_name() const { return name_; }

  // TODO: retrun bool to inform result of mode switching to the upper class
  virtual void starting() = 0;

  virtual void read(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  virtual void write(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  const std::string name_;
  const std::shared_ptr<DynamixelActuatorContext> context_;
};

} // namespace layered_hardware_dynamixel

#endif