#include <boost/bind.hpp>
#include <ros/console.h>
#include <ros/ros.h>

#include "reflex_hand.h"
#include "reflex_driver.h"

int main(int argc, char **argv) {

  // Initialize ROS node
  ros::init(argc, argv, "hand1_reflex_driver_node");
  ros::NodeHandle nh, nh_private("~");

  //////////////////////////////////////////////////////////
  // Hand 1
  std::string hand1_network_interface = "eth0";
  std::string hand1_prefix = "hand1";
  std::string ns1 = "/reflex_takktile";
  char* hand1_mcast_addr = "224.0.0.124";
  int hand1_pb = 11333;
  ROS_INFO("checking interface hand1_reflex_driver_node: %s", hand1_network_interface.c_str());
  boost::shared_ptr<reflex_driver::ReflexDriver> hand1_driver(boost::make_shared<reflex_driver::ReflexDriver>(hand1_network_interface, hand1_pb, hand1_prefix, hand1_mcast_addr));

  // Initialize the hand command services
  ros::ServiceServer hand1_set_speed_service =
    nh.advertiseService(ns1 + "/set_speed", &reflex_driver::ReflexDriver::set_motor_speed, hand1_driver);
    ROS_INFO("Advertising the %s/set_speed service", ns1.c_str());
    
  ros::ServiceServer hand1_disable_service = 
    nh.advertiseService(ns1 + "/disable_torque", &reflex_driver::ReflexDriver::disable_torque, hand1_driver);
  ROS_INFO("Advertising the %s/disable_torque service", ns1.c_str());

  // Initialize the /calibrate_tactile and /calibrate_fingers services
  ros::ServiceServer hand1_calibrate_fingers_service = 
    nh.advertiseService(ns1 + "/calibrate_fingers", &reflex_driver::ReflexDriver::calibrate_fingers, hand1_driver);
  ROS_INFO("Advertising the %s/calibrate_fingers service", ns1.c_str());

  ros::ServiceServer hand1_calibrate_tactile_service = 
    nh.advertiseService(ns1 + "/calibrate_tactile", &reflex_driver::ReflexDriver::calibrate_tactile, hand1_driver);
  ROS_INFO("Advertising the %s/calibrate_tactile service", ns1.c_str());

  ros::ServiceServer hand1_set_thresh_service = 
    nh.advertiseService(ns1 + "/set_tactile_threshold", &reflex_driver::ReflexDriver::set_tactile_threshold, hand1_driver);
  ROS_INFO("Advertising the %s/set_tactile_threshold service", ns1.c_str());

  ros::Subscriber hand1_raw_positions_sub =
    nh.subscribe<reflex_msgs::RawServoCommands>(ns1 + "/raw_hand_command", 10, &reflex_driver::ReflexDriver::receive_raw_cmd_cb, hand1_driver);
  ros::Subscriber hand1_radian_positions_sub =
    nh.subscribe<reflex_msgs::RadianServoCommands>(ns1 + "/radian_hand_command", 10, &reflex_driver::ReflexDriver::receive_angle_cmd_cb, hand1_driver);


  ROS_INFO("Entering main hand1_reflex_driver loop...");
  while (ros::ok()) {
    ros::spinOnce();
    if (!(hand1_driver->rh.listen(0.001)))
      ROS_ERROR("Error in listen");
  }

  hand1_driver->rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  ros::Duration(0.01).sleep();
  ROS_INFO("Have a nice day");
  
  return 0;
}