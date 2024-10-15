#ifndef LAYERED_HARDWARE_DYNAMIXEL_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LHD_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware_dynamixel"), __VA_ARGS__)
#define LHD_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware_dynamixel"), __VA_ARGS__)
#define LHD_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware_dynamixel"), __VA_ARGS__)
#define LHD_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware_dynamixel"), __VA_ARGS__)
#define LHD_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware_dynamixel"), __VA_ARGS__)

#endif