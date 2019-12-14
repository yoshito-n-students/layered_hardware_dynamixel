#include <controller_manager/controller_manager.h>
#include <dynamixel_hardware/dynamixel_hardware.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dynamixel_hardware_node");
  ros::NodeHandle nh, pnh("~");

  dynamixel_hardware::DynamixelHardware hw;
  if (hw.init(pnh)) {
    ROS_ERROR("Failed to init DynamixelHardware");
    return 1;
  }

  controller_manager::ControllerManager cm(&hw, nh);

  ros::Rate rate(pnh.param("control_frequency", 0.1));
  ros::Time prev_time(ros::Time::now());
  while (ros::ok()) {
    const ros::Time time(ros::Time::now());
    const ros::Duration period(time - prev_time);
    hw.read(time, period);
    cm.update(time, period);
    hw.write(time, period);
    prev_time = time;
    rate.sleep();
  }

  return 0;
}