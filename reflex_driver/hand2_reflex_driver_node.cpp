#include <boost/bind.hpp>
#include <ros/console.h>
#include <ros/ros.h>

#include "reflex_hand.h"
#include "reflex_driver.h"

int main(int argc, char **argv) {

  // Initialize ROS node
  ros::init(argc, argv, "hand2_reflex_driver_node");
  ros::NodeHandle nh, nh_private("~");

  /////////////////////////////////////////////////////////
  // Hand 2
  std::string hand2_network_interface = "eth0";
  std::string hand2_prefix = "hand2";
  std::string ns2 = "/" + hand2_prefix + "/reflex_takktile";
  char* hand2_mcast_addr = "224.0.0.126";
  int hand2_pb = 11334;
  ROS_INFO("checking interface hand2_reflex_driver_node: %s", hand2_network_interface.c_str());

  boost::shared_ptr<reflex_driver::ReflexDriver> hand2_driver(boost::make_shared<reflex_driver::ReflexDriver>(hand2_network_interface, hand2_pb, hand2_prefix, hand2_mcast_addr));

  // Initialize the hand command services
  ros::ServiceServer hand2_set_speed_service =
    nh.advertiseService(ns2 + "/set_speed", &reflex_driver::ReflexDriver::set_motor_speed, hand2_driver);
    ROS_INFO("Advertising the %s/set_speed service", ns2.c_str());
    
  ros::ServiceServer hand2_disable_service = 
    nh.advertiseService(ns2 + "/disable_torque", &reflex_driver::ReflexDriver::disable_torque, hand2_driver);
  ROS_INFO("Advertising the %s/disable_torque service", ns2.c_str());

  // Initialize the /calibrate_tactile and /calibrate_fingers services
  ros::ServiceServer hand2_calibrate_fingers_service = 
    nh.advertiseService(ns2 + "/calibrate_fingers", &reflex_driver::ReflexDriver::calibrate_fingers, hand2_driver);
  ROS_INFO("Advertising the %s/calibrate_fingers service", ns2.c_str());

  ros::ServiceServer hand2_calibrate_tactile_service = 
    nh.advertiseService(ns2 + "/calibrate_tactile", &reflex_driver::ReflexDriver::calibrate_tactile, hand2_driver);
  ROS_INFO("Advertising the %s/calibrate_tactile service", ns2.c_str());

  ros::ServiceServer hand2_set_thresh_service = 
    nh.advertiseService(ns2 + "/set_tactile_threshold", &reflex_driver::ReflexDriver::set_tactile_threshold, hand2_driver);
  ROS_INFO("Advertising the %s/set_tactile_threshold service", ns2.c_str());

  ros::Subscriber hand2_raw_positions_sub =
    nh.subscribe<reflex_msgs::RawServoCommands>(ns2 + "/raw_hand_command", 10, &reflex_driver::ReflexDriver::receive_raw_cmd_cb, hand2_driver);
  ros::Subscriber hand2_radian_positions_sub =
    nh.subscribe<reflex_msgs::RadianServoCommands>(ns2 + "/radian_hand_command", 10, &reflex_driver::ReflexDriver::receive_angle_cmd_cb, hand2_driver);


  ROS_INFO("Entering main hand2_reflex_driver loop...");
  while (ros::ok()) {
    ros::spinOnce();
    if (!(hand2_driver->rh.listen(.001)))
      ROS_ERROR("Error in listen");
  }

  hand2_driver->rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  ros::Duration(0.01).sleep();
  ROS_INFO("Have a nice day");
  
  return 0;
}