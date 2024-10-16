#ifndef LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_DYNAMIXEL_ACTUATOR_HPP

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware_dynamixel/clear_multi_turn_mode.hpp>
#include <layered_hardware_dynamixel/common_namespaces.hpp>
#include <layered_hardware_dynamixel/current_based_position_mode.hpp>
#include <layered_hardware_dynamixel/current_mode.hpp>
#include <layered_hardware_dynamixel/dynamixel_actuator_context.hpp>
#include <layered_hardware_dynamixel/dynamixel_workbench_utils.hpp>
#include <layered_hardware_dynamixel/extended_position_mode.hpp>
#include <layered_hardware_dynamixel/logging_utils.hpp>
#include <layered_hardware_dynamixel/operating_mode_interface.hpp>
#include <layered_hardware_dynamixel/position_mode.hpp>
#include <layered_hardware_dynamixel/reboot_mode.hpp>
#include <layered_hardware_dynamixel/torque_disable_mode.hpp>
#include <layered_hardware_dynamixel/velocity_mode.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_dynamixel {

class DynamixelActuator {
public:
  DynamixelActuator(const std::string &name, const YAML::Node &params,
                    const std::shared_ptr<DynamixelWorkbench> &dxl_wb) {
    // parse parameters for this actuator
    std::uint8_t id;
    double torque_constant;
    std::vector<std::string> mapped_mode_names;
    try {
      id = static_cast<std::uint8_t>(params["id"].as<int>());
      torque_constant = params["torque_constant"].as<double>();
      for (const auto &iface_mode_name_pair : params["operating_mode_map"]) {
        bound_interfaces_.emplace_back(iface_mode_name_pair.first.as<std::string>());
        mapped_mode_names.emplace_back(iface_mode_name_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameters for \"" + name +
                               "\" actuator: " + error.what());
    }

    // allocate context
    context_.reset(new DynamixelActuatorContext{name, dxl_wb, id, torque_constant});

    // find dynamixel actuator by id
    if (!ping(context_)) {
      std::ostringstream msg;
      msg << "Failed to ping the actuator \"" << name << "\" actuator (id: " << static_cast<int>(id)
          << ")";
      throw std::runtime_error(msg.str());
    }

    // make operating mode map from ros-controller name to dynamixel's operating mode
    for (const auto &mode_name : mapped_mode_names) {
      try {
        mapped_modes_.emplace_back(make_operating_mode(mode_name));
      } catch (const std::runtime_error &error) {
        throw std::runtime_error("Invalid value in \"operating_mode_map\" parameter for \"" + name +
                                 "\" actuator: " + error.what());
      }
    }
  }

  virtual ~DynamixelActuator() {
    // finalize the present mode
    switch_operating_modes(/* new_mode = */ nullptr);
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to actuator states owned by this actuator
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to actuator commands owned by this actuator
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff_cmd);
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      LHD_ERROR("DynamixelActuator::prepare_command_mode_switch(): "
                "Reject mode switching of \"%s\" actuator "
                "because %zd bound interfaces are about to be active",
                context_->name.c_str(), active_bound_ifaces.size());
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      LHD_ERROR("DynamixelActuator::perform_command_mode_switch(): "
                "Could not switch mode of \"%s\" actuator "
                "because %zd bound interfaces are active",
                context_->name.c_str(), bound_interfaces_.size());
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    if (!active_bound_ifaces.empty()) { // active_bound_ifaces.size() == 1
      switch_operating_modes(mapped_modes_[active_bound_ifaces.front()]);
    } else { // active_bound_ifaces.size() == 0
      switch_operating_modes(nullptr);
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
    return hi::return_type::OK; // TODO: return result of read
  }

  hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
    return hi::return_type::OK; // TODO: return result of write
  }

private:
  std::shared_ptr<OperatingModeInterface> make_operating_mode(const std::string &mode_str) const {
    if (mode_str == "clear_multi_turn") {
      return std::make_shared<ClearMultiTurnMode>(context_);
    } else if (mode_str == "current") {
      return std::make_shared<CurrentMode>(context_);
    } else if (mode_str == "current_based_position") {
      return std::make_shared<CurrentBasedPositionMode>(context_);
    } else if (mode_str == "extended_position") {
      return std::make_shared<ExtendedPositionMode>(context_);
    } else if (mode_str == "position") {
      return std::make_shared<PositionMode>(context_);
    } else if (mode_str == "reboot") {
      return std::make_shared<RebootMode>(context_);
    } else if (mode_str == "torque_disable") {
      return std::make_shared<TorqueDisableMode>(context_);
    } else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(context_);
    } else {
      throw std::runtime_error("Unknown operating mode name \"" + mode_str + "\"");
    }
  }

  void switch_operating_modes(const std::shared_ptr<OperatingModeInterface> &new_mode) {
    // do nothing if no mode switch is requested
    if (present_mode_ == new_mode) {
      return;
    }
    // stop present mode
    if (present_mode_) {
      LHD_INFO("DynamixelActuator::switch_operating_modes(): "
               "Stopping \"%s\" operating mode for \"%s\" actuator",
               present_mode_->get_name().c_str(), context_->name.c_str());
      present_mode_->stopping();
      present_mode_.reset();
    }
    // start new mode
    if (new_mode) {
      LHD_INFO("DynamixelActuator::switch_operating_modes(): "
               "Starting \"%s\" operating mode for \"%s\" actuator",
               new_mode->get_name().c_str(), context_->name.c_str());
      new_mode->starting();
      present_mode_ = new_mode;
    }
  }

private:
  std::shared_ptr<DynamixelActuatorContext> context_;

  // present operating mode
  std::shared_ptr<OperatingModeInterface> present_mode_;
  // map from command interface to operating mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<std::shared_ptr<OperatingModeInterface>> mapped_modes_;
};

} // namespace layered_hardware_dynamixel

#endif