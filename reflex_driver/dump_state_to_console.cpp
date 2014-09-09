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

typedef struct
{
  uint8_t  header[4];
  uint32_t systime;
  uint16_t tactile_pressures[ReflexHandState::NUM_TACTILE];
  uint16_t tactile_temperatures[ReflexHandState::NUM_TACTILE];
  uint16_t encoders[ReflexHandState::NUM_FINGERS];
  uint8_t  dynamixel_error_states[4];
  uint16_t dynamixel_angles[4];
  uint16_t dynamixel_speeds[4];
  uint16_t dynamixel_loads[4];
  uint8_t  dynamixel_voltages[4];
  uint8_t  dynamixel_temperatures[4];
} __attribute__((packed)) mcu_state_format_1_t;


void reflex_hand_state_cb(const reflex_hand::ReflexHandState * const s)
{
  ROS_INFO("rhs_cb");
  //mcu_state_format_1_t *s = (mcu_
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
