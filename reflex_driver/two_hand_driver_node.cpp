#include <boost/bind.hpp>
#include <ros/console.h>
#include <ros/ros.h>

#include "reflex_hand.h"
#include "reflex_driver.h"

int main(int argc, char **argv) {

  // TODO: add in a second hand after 1 hand works

  // Initialize ROS node
  ros::init(argc, argv, "/reflex_driver");
  ros::NodeHandle nh, nh_private("~");

  std::string network_interface = "eth0";
  std::string hand_prefix = "hand1";

  ROS_INFO("Trying to instantiate ReflexDriver");

  //boost::shared_ptr<reflex_driver::ReflexDriver> driver(boost::make_shared<reflex_driver::ReflexDriver>(network_interface, hand_prefix));

  reflex_driver::ReflexDriver driver(network_interface, hand_prefix);

  ROS_INFO("Successfully instantiated ReflexDriver");

  ROS_INFO("Entering main reflex_driver loop...");
  while (ros::ok()) {
    ros::spinOnce();
    if (!driver->rh.listen(0.001))
      ROS_ERROR("Error in listen");
  }

  driver->rh.setServoControlModes(reflex_hand::ReflexHand::CM_IDLE);
  ros::Duration(0.01).sleep();
  ROS_INFO("Have a nice day");
  
  // driver.rh->setStateCallback(boost::bind(&reflex_driver::ReflexDriver::reflex_hand_state_cb, &driver, state)); // TODO: hopefully remove
  
  return 0;
}