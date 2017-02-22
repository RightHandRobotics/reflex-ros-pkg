/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2014 Open Source Robotics Foundation, Inc.
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
/////////////////////////////////////////////////////////////////////////////

#include "reflex_hand.h"
#include <ros/console.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <netdb.h>
using namespace reflex_hand;

/////////////////////////////////////////////////////////////////////////

ReflexHandState::ReflexHandState()
{
  systime_us_ = 0;
  for (int i = 0; i < NUM_TAKKTILE; i++)
    tactile_pressures_[i] = tactile_temperatures_[i] = 0;
}

/////////////////////////////////////////////////////////////////////////

ReflexHand::ReflexHand(const std::string &interface, int pb, const char* &mcast_addr_str)
: happy_(true)
{
  ROS_INFO("ReflexHand constructor");
  ROS_INFO("checking interface: %s", interface.c_str());
  //const char *mcast_addr_str = "224.0.0.124"; // parameterize someday !
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_FATAL_COND(tx_sock_ < 0, "couldn't create socket");
  ROS_FATAL_COND(rx_sock_ < 0, "couldn't create socket");
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_addr_str);
  PORT_BASE = pb;

  ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1)
  {
    ROS_FATAL("couldn't get ipv4 address of interface %s", interface.c_str());
    return;
  }
  std::string tx_iface_addr;
  bool found_interface = false;
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr)
      continue;
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET)
      continue;
    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                    host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;
    ROS_INFO("found address %s on interface %s",
             host, ifa->ifa_name);
    if (std::string(ifa->ifa_name) == interface)
    {
      ROS_INFO("using %s as the tx interface for IPv4 UDP multicast", host);
      tx_iface_addr = host;
      found_interface = true;
      break;
    }
  }
  freeifaddrs(ifaddr);
  if (!found_interface)
  {
    ROS_FATAL("Unable to find IPv4 address of interface %s. Perhaps it needs to be set to a static address?", interface.c_str());
    happy_ = false;
    return;
  }

  in_addr local_addr;
  local_addr.s_addr = inet_addr(tx_iface_addr.c_str());
  int result = 0, loopback = 0;
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF,
                      (char *)&local_addr, sizeof(local_addr));
  ROS_FATAL_COND(result < 0, "couldn't set local interface for udp tx sock, fails with errno: %d", errno);
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_LOOP, 
                      &loopback, sizeof(loopback));
  ROS_FATAL_COND(result < 0, "couldn't turn off outgoing multicast loopback, fails with errno: %d", errno);

  /////////////////////////////////////////////////////////////////////
  // set up the rx side of things
  int reuseaddr = 1;
  result = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
                      &reuseaddr, sizeof(reuseaddr));
  ROS_FATAL_COND(result < 0, 
                 "couldn't set SO_REUSEADDR on UDP RX socket, "
                 "fails with errno: %d", errno);
  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  rx_bind_addr.sin_port = htons(PORT_BASE);
  result = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  ROS_FATAL_COND(result < 0, 
                 "couldn't bind rx socket to port %d, fails with errno: %d", 
                 PORT_BASE, errno);
  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_str);
  mreq.imr_interface.s_addr = inet_addr(tx_iface_addr.c_str());
  result = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      &mreq, sizeof(mreq));
  ROS_FATAL_COND(result < 0, 
                 "couldn't add to multicast group, fails with errno: %d", 
                 errno);
  ROS_INFO("constructor complete");
}


ReflexHand::~ReflexHand()
{
}

void ReflexHand::tx(const uint8_t *msg, 
                    const uint16_t msg_len, 
                    const uint16_t port)
{
  // ROS_INFO("ReflexHand::tx %d bytes to port %d", msg_len, port); // Check or uncheck for debugging
  mcast_addr_.sin_port = htons(port);
  int nsent = sendto(tx_sock_, msg, msg_len, 0,
                     (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
}

bool ReflexHand::listen(const double max_seconds)
{
  static uint8_t rxbuf[2000] = {0};
  fd_set rdset;
  FD_ZERO(&rdset);
  FD_SET(rx_sock_, &rdset);
  timeval timeout;
  timeout.tv_sec = (time_t)trunc(max_seconds);
  timeout.tv_usec = (suseconds_t)((max_seconds - timeout.tv_sec) * 1e6);
  int rv = select(rx_sock_ + 1, &rdset, NULL, NULL, &timeout);

  if (rv > 0 && FD_ISSET(rx_sock_, &rdset))
  {
    int nbytes = recvfrom(rx_sock_, rxbuf, sizeof(rxbuf), 0, NULL, NULL);
    rx(rxbuf, nbytes);
  }
 
  /* 
  if (rv < 0)
    return false;
  if (rv == 0 || !FD_ISSET(rx_sock_, &rdset))
    return true; // nothing happened; hit timeout
  if (nbytes < 0)
    return false;
  if (state_cb_)
    state_cb_(&rx_state_);
  */ 
  return true;
}

void ReflexHand::setServoTargets(const uint16_t *targets)
{
  uint8_t msg[1 + 2*NUM_SERVOS];
  msg[0] = CP_SET_SERVO_TARGET;
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    msg[1 + 2*i] = (targets[i] >> 8) & 0xff;
    msg[2 + 2*i] = targets[i] & 0xff;
  }
  tx(msg, sizeof(msg), PORT_BASE);
}

void ReflexHand::setServoControlModes(const ControlMode *modes)
{
  uint8_t msg[NUM_SERVOS+1];
  msg[0] = CP_SET_SERVO_MODE; 
  for (int i = 0; i < NUM_SERVOS; i++)
    msg[i+1] = (uint8_t)modes[i];
  tx(msg, sizeof(msg), PORT_BASE);
}

void ReflexHand::setServoControlModes(const ControlMode mode)
{
  const ControlMode modes[4] = { mode, mode, mode, mode };
  setServoControlModes(modes);
}

typedef struct
{
  uint8_t  header[4];
  uint32_t systime;
  uint16_t tactile_pressures[NUM_TAKKTILE];
  uint16_t tactile_temperatures[NUM_TAKKTILE];
  uint16_t encoders[NUM_FINGERS];
  uint8_t  dynamixel_error_states[4];
  uint16_t dynamixel_angles[4];
  uint16_t dynamixel_speeds[4];
  uint16_t dynamixel_loads[4];
  uint8_t  dynamixel_voltages[4];
  uint8_t  dynamixel_temperatures[4];
} __attribute__((packed)) mcu_state_format_1_t;

void ReflexHand::rx(const uint8_t *msg, const uint16_t msg_len)
{
  // first, check the packet format "magic byte" and the length
  if (msg[0] != 1)
  {
    ROS_ERROR("unexpected magic byte received on UDP multicast port: 0x%02x",
              msg[0]);
    return;
  }
  if (msg_len != sizeof(mcu_state_format_1_t))  // The leftover palm data adds 44 bytes
  {
    ROS_ERROR("expected packet length %d, but saw %d instead",
              (int)sizeof(mcu_state_format_1_t), msg_len - 44);
    return;
  }
  mcu_state_format_1_t *rx_state_msg = (mcu_state_format_1_t *)msg;
  rx_state_.systime_us_ = rx_state_msg->systime;
  for (int i = 0; i < NUM_FINGERS; i++)
    rx_state_.encoders_[i] = rx_state_msg->encoders[i];
  for (int i = 0; i < NUM_TAKKTILE; i++)
  {
    rx_state_.tactile_pressures_[i]    = rx_state_msg->tactile_pressures[i];
    rx_state_.tactile_temperatures_[i] = rx_state_msg->tactile_temperatures[i];
  }
  for (int i = 0; i < 4; i++)
  {
    rx_state_.dynamixel_error_states_[i] = 
      rx_state_msg->dynamixel_error_states[i];
    rx_state_.dynamixel_angles_[i]   = rx_state_msg->dynamixel_angles[i];
    rx_state_.dynamixel_speeds_[i]   = rx_state_msg->dynamixel_speeds[i];
    rx_state_.dynamixel_loads_[i]    = rx_state_msg->dynamixel_loads[i];
    rx_state_.dynamixel_voltages_[i] = rx_state_msg->dynamixel_voltages[i];
    rx_state_.dynamixel_temperatures_[i] = rx_state_msg->dynamixel_temperatures[i];
  }
  // now that we have stuff the rx_state_ struct, fire off our callback
  //ROS_INFO("Received data from port base %d", PORT_BASE);
  if (state_cb_)
    state_cb_(&rx_state_);
}

void ReflexHand::setStateCallback(StateCallback callback)
{
  state_cb_ = callback;
}
