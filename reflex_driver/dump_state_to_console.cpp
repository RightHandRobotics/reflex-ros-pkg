#include <ros/ros.h>
#include <signal.h>
#include <string>
#include "reflex_hand.h"
using std::string;
using reflex_hand::ReflexHandState;

static bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}

void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const s)
{
  ROS_INFO("rhs_cb");
  printf("systime_us = %u\n", (unsigned)s->systime_us_);
  printf("tactile pressures:\n  ");
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_TACTILE; i++)
  {
    printf("%5hu ", s->tactile_pressures_[i]);
    if (i != 0 && i < 30 && i % 9 == 8)
      printf("\n  ");
  }
  printf("\n");

  printf("tactile temperatures:\n  ");
  for (int i = 0; i < reflex_hand::ReflexHandState::NUM_TACTILE; i++)
  {
    printf("%5hu ", s->tactile_temperatures_[i]);
    if (i != 0 && i < 30 && i % 9 == 8)
      printf("\n  ");
  }
  printf("\n");

  printf("dynamixel:\n");
  for (int i = 0; i < 4; i++)
    printf("  %6hu  %6hu  %6hu  %6hu  %6hu\n", 
           s->dynamixel_angles_[i],
           s->dynamixel_speeds_[i],
           s->dynamixel_loads_[i],
           s->dynamixel_voltages_[i],
           s->dynamixel_temperatures_[i]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reflex_hand_dump_state");
  ros::NodeHandle nh, nh_private("~");
  string network_interface;
  nh_private.param<string>("network_interface", network_interface, "eth0");
  reflex_hand::ReflexHand rh(network_interface);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  if (!rh.happy())
  {
    ROS_FATAL("error during ReflexHand initialization. Have a nice day.");
    return 1;
  }
  rh.setStateCallback(reflex_hand_state_cb);
  while (!g_done)
  {
    if (!rh.listen(0.001))
    {
      ROS_ERROR("error in listen");
      break;
    }
  }
  ROS_INFO("have a nice day.");
  return 0;
}
